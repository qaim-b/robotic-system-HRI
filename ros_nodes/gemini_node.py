#!/usr/bin/env python3
from gtts import gTTS
import tempfile 
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger
import os
import google.generativeai as genai
import csv
from datetime import datetime
from emotion_speech.config import USE_MEMORY, USE_EMOTION
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import random
import concurrent.futures  # [APPENDED] for watchdog
import traceback

MEMORY_DB_PATH = '/home/qaim/Desktop/ros2_ws/src/emotion_speech/dashboard/memory.db'
print(f"[GEMINI_NODE] Using memory database at: {MEMORY_DB_PATH}")

class GeminiResponder(Node):
    def __init__(self):
        super().__init__('gemini_responder')

        genai.configure(api_key=os.environ["GEMINI_API_KEY"])
        self.model = genai.GenerativeModel("models/gemini-1.5-flash-latest")

        self.user_id = 'default_user'
        self.turn_index = 1
        self.current_valence = 0.0

        self.session = "1"
        self.phase = "Unknown Phase"
        self.current_emotion = None

        self._stt_enabled = True
        self._capturing = False
        self._captured_text = []
        self._capture_timer = None
        self.last_heard = None
        self._speech_timer = None
        self._speaking_lock = threading.Lock()
        self._last_bot_reply = None

        self.create_subscription(String, 'emotion_state', self.emotion_callback, 10)
        self.create_subscription(String, 'human_speech', self.human_speech_callback, 10)
        self.create_subscription(String, 'current_session_id', self.session_callback, 10)
        self.create_subscription(String, 'experiment_phase', self.phase_callback, 10)
        self.create_subscription(String, 'experiment_pattern', self.pattern_callback, 10)

        self.latest_valence = None
        self.create_subscription(String, 'pnnx2', self.pnnx2_callback, 10)

        qos_profile = QoSProfile(depth=1)
        self.create_subscription(String, 'current_user_id', self.user_id_callback, qos_profile=qos_profile)

        self.mem_cli = self.create_client(Trigger, 'get_context_memories')
        while not self.mem_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_context_memories service...')
        self.mem_req = Trigger.Request()

        self.speak_pub = self.create_publisher(String, 'speak_emotion', 10)
        self.memory_pub = self.create_publisher(String, 'memory_log', 14)

        self.get_logger().info("Gemini responder node started; waiting for input...")

        self._pending_user_utterance = None
        self._pending_mood = None

        print(f"[INFO] Pattern active for this session: USE_MEMORY={USE_MEMORY}, USE_EMOTION={USE_EMOTION}")
        self.pattern = "Vanilla"

    def infer_pattern_flags(self, pattern_label):
        label = (pattern_label or "").lower()
        return 1 if "memory" in label else 0

    def pnnx2_callback(self, msg):
        try:
            self.latest_valence = float(msg.data.strip())
        except Exception:
            self.latest_valence = None

    def pattern_callback(self, msg):
        self.pattern = msg.data
        print(f"[GEMINI_NODE] Pattern set to: {self.pattern}")

    def session_callback(self, msg):
        self.session = msg.data
        print(f"[GEMINI_NODE] Session set to: {self.session}")

    def phase_callback(self, msg):
        self.phase = msg.data
        print(f"[GEMINI_NODE] Phase set to: {self.phase}")
        if self.phase.lower().startswith("baseline") or self.phase.lower().startswith("reflection"):
            self.log_baseline_reflection_row()

    def log_baseline_reflection_row(self):
        CSV_LOG_FILE = '/home/qaim/Desktop/ros2_ws/baseline_reflection_log.csv'
        valence_value = self.latest_valence if self.latest_valence is not None else ""
        write_header = not os.path.exists(CSV_LOG_FILE)
        with open(CSV_LOG_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(['timestamp', 'phase', 'pattern', 'user_id', 'session', 'valence'])
            writer.writerow([
                datetime.now().isoformat(),
                self.phase,
                self.pattern,
                self.user_id,
                self.session,
                valence_value
            ])
        print(f"[LOG] Baseline/Reflection row written: phase={self.phase}, user_id={self.user_id}, session={self.session}, pattern={self.pattern}, valence={valence_value}")

    def user_id_callback(self, msg):
        print(f"[GEMINI_NODE] Received user_id on topic: '{msg.data}'")
        self.user_id = msg.data

    def log_turn_to_csv(self, timestamp, pattern, user_utterance, robot_reply, turn_index, valence, emotion_label, used_memory, category):
        CSV_LOG_FILE = '/home/qaim/Desktop/ros2_ws/conversation_log.csv'
        log_row = [
            timestamp,
            self.phase,
            pattern,
            self.user_id,
            self.session,
            turn_index,
            user_utterance,
            robot_reply,
            valence,
            emotion_label,
            used_memory,
            category,
        ]
        write_header = not os.path.exists(CSV_LOG_FILE)
        with open(CSV_LOG_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow([
                    'timestamp', 'phase', 'pattern', 'user_id', 'session', 'turn_index',
                    'user_utterance', 'robot_reply', 'valence', 'emotion_label',
                    'used_memory', 'category'
                ])
            writer.writerow(log_row)

    def emotion_callback(self, msg):
        try:
            val = float(msg.data.strip())
        except ValueError:
            self.get_logger().warning(f"Ignoring non-float emotion_state: {msg.data}")
            return

        self.current_valence = val
        if val >= 0.8:
            label = "extremely comfortable"
        elif val >= 0.6:
            label = "very comfortable"
        elif val >= 0.4:
            label = "comfortable"
        elif val >= 0.2:
            label = "content"
        else:
            label = "uncomfortable"

        self.current_emotion = label
        self.get_logger().info(f"[EMOTION] {self.current_emotion} (valence: {self.current_valence:.3f})")
        self.speak_pub.publish(String(data=self.current_emotion))

    def human_speech_callback(self, msg):
        text = msg.data.strip()
        if self._last_bot_reply and self._last_bot_reply.lower() in text.lower():
            return
        if self._speaking_lock.locked():
            return
        if not text or not self._stt_enabled or self.current_emotion is None:
            return
        if not self._capturing:
            self._capturing     = True
            self._captured_text = []
            self.get_logger().info("🔴 The capture of user speech has ended")
            threading.Thread(target=self._countdown, args=(1,), daemon=True).start()
            self._capture_timer = threading.Timer(1.0, self._on_capture_complete)
            self._capture_timer.start()
        self._captured_text.append(text)

    def _countdown(self, seconds):
        for remaining in range(seconds, 0, -1):
            self.get_logger().info(f"⏳ Capture ends in {remaining}s…")
            time.sleep(1)
        self.get_logger().info("✅ Capture window ended")

    def _on_capture_complete(self):
        self._stt_enabled = False
        full_text = ' '.join(self._captured_text)
        self.get_logger().info(f"[CAPTURE COMPLETE] \"{full_text}\"")
        self._process_user_speech(full_text)

    def fetch_context_from_memory_service(self, user_utterance, mood):
        future = self.mem_cli.call_async(self.mem_req)
        self._pending_user_utterance = user_utterance
        self._pending_mood = mood
        future.add_done_callback(self.on_memory_service_response)

    def on_memory_service_response(self, future):
        try:
            context_string = ""
            result = future.result()
            if result is not None and getattr(result, 'success', True):
                context_string = result.message if hasattr(result, 'message') else ""
        except Exception as e:
            self.get_logger().error(f"Memory service call failed: {e}")
            context_string = ""
        self._process_gemini_response(self._pending_user_utterance, self._pending_mood, context_string)
        self._pending_user_utterance = None
        self._pending_mood = None

    def _process_user_speech(self, user_utterance):
        mood = self.current_emotion if self.current_emotion else "unknown"
        if "memory" in self.pattern.lower():
            self.fetch_context_from_memory_service(user_utterance, mood)
        else:
            self._process_gemini_response(user_utterance, mood, "")

    # ==================== [APPENDED] Watchdog + Fallback ====================

    def _call_gemini_with_watchdog(self, prompt: str, timeout_s: int = 15):
        """Run Gemini API call with timeout to avoid hangs (15s)."""
        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as ex:
            fut = ex.submit(self.model.generate_content, prompt)
            try:
                return fut.result(timeout=timeout_s)
            except concurrent.futures.TimeoutError:
                self.get_logger().error(f"[LLM] TIMEOUT after {timeout_s}s (watchdog)")
                return None
            except Exception as e:
                self.get_logger().error(f"[LLM] ERROR (watchdog): {type(e).__name__}: {e}")
                traceback.print_exc()
                return None

    def _fallback_reply(self, user_text: str) -> str:
        """Context-aware fallback reply so robot never goes silent."""
        followup = " Want me to ask you something else meanwhile?"
        txt = user_text.lower()
        if "weather" in txt:
            return ("I’m having trouble reaching my brain right now, but generally "
                    "Malaysia is warm and humid with frequent rain. Tell me your city and I’ll try again." + followup)
        return ("I’m having a little trouble reaching my brain right now, but I’m still here with you. "
                "You can rephrase, or ask me something different." + followup)

    # ==================== [KEEPING V1 PROMPT ENGINEERING] ====================

    def _process_gemini_response(self, user_utterance, mood, context_string):
        context_lines = []
        key_facts_lines = []
        if "\nUser facts and interests:\n" in context_string:
            context_part, facts_part = context_string.split("\nUser facts and interests:\n", 1)
            context_lines = [l.strip() for l in context_part.split('\n\n') if l.strip() and not l.startswith('No context')]
            key_facts_lines = [l.strip() for l in facts_part.strip().splitlines() if l.strip()]
        else:
            context_lines = [l.strip() for l in context_string.split('\n\n') if l.strip() and not l.startswith('No context')]
            key_facts_lines = []

        num_memories_used = len(context_lines)
        used_memory = num_memories_used

        # Build rich prompt (v1 style)
        prompt = "You are an open-minded, friendly robot designed for emotionally intelligent, memory-driven conversation with humans.\n"
        if "memory" in self.pattern.lower() and context_lines:
            prompt += "\nHere is the recent conversation history:\n" + "\n".join(context_lines) + "\n"
        if key_facts_lines:
            prompt += "\nHere are important facts and interests about the user:\n" + "\n".join(key_facts_lines) + "\n"
        if "emotion" in self.pattern.lower() and self.current_emotion:
            prompt += f"\nThe user is feeling {self.current_emotion} (valence={self.current_valence:.2f}).\n"
        prompt += f'\nUser just said: "{user_utterance}"\n'
        prompt += (
            "\nReply kindly and contextually. You may reply in one, two, or three sentences depending on the user's needs, "
            "the natural flow of conversation, or if you want to offer information and ask a friendly question.\n"
            "If the user seems to like something (such as a food, activity, or place), feel free to make proactive suggestions or recommendations related to that interest.\n"
        )
        if random.random() < 0.3:
            prompt += "After your main reply, ask the user a relevant, friendly follow-up question.\n"

        self.get_logger().info(f"[PROMPT] {prompt}")

        timestamp = datetime.now().isoformat()
        category = "normal"
        robot_reply = ""

        try:
            self.get_logger().info("[LLM] Calling Gemini…")
            response = self._call_gemini_with_watchdog(prompt, timeout_s=15)
            robot_reply = (response.text or "").strip() if response and hasattr(response, "text") else ""
            if not robot_reply:
                self.get_logger().warning("[LLM] Empty response, using fallback")
                robot_reply = self._fallback_reply(user_utterance)
            self._last_bot_reply = robot_reply
            self.get_logger().info(f"[LLM] Reply: {robot_reply[:160]}…")

            # Log + TTS
            self._speak_and_log(user_utterance, robot_reply, timestamp, used_memory, category)

        except Exception as e:
            self.get_logger().error(f"[LLM/TTS] ERROR: {e}")
            robot_reply = self._fallback_reply(user_utterance)
            try:
                self._speak_and_log(user_utterance, robot_reply, timestamp, used_memory, category)
            except Exception as e2:
                self.get_logger().error(f"[TTS Fallback] ERROR: {e2}")

        finally:
            self.get_logger().info("🎤 Response done—STT will re-enable in 1s")
            self._capturing = False
            self.turn_index += 1
            threading.Timer(1.0, self._enable_stt).start()

    # ==================== [APPENDED TTS PLAYBACK FROM V2] ====================

    def _speak_and_log(self, user_utterance, robot_reply, timestamp, used_memory, category):
        if not self._speaking_lock.acquire(blocking=False):
            self.get_logger().info("🛑 Still speaking—skipping this utterance")
            return
        try:
            self.get_logger().info("[TTS] start gTTS")
            tts = gTTS(text=robot_reply, lang='en')
            with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as fp:
                tts.save(fp.name)
                subprocess.run(["mpg123", fp.name])
            self.get_logger().info("[TTS] done playback")
        finally:
            self._speaking_lock.release()

        self.log_turn_to_csv(
            timestamp, self.pattern, user_utterance, robot_reply, self.turn_index,
            self.current_valence, self.current_emotion, used_memory, category
        )

    def _enable_stt(self):
        self._stt_enabled = True
        self.get_logger().info("🔓 STT re-enabled")

def main(args=None):
    rclpy.init(args=args)
    node = GeminiResponder()
    try:
        rclpy.spin(node)
    finally:
        if node._capture_timer:
            node._capture_timer.cancel()
        if node._speech_timer:
            node._speech_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
