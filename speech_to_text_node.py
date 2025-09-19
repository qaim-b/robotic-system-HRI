import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import json
from vosk import Model, KaldiRecognizer
import threading
import tkinter as tk

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        self.publisher_ = self.create_publisher(String, 'human_speech', 10)

        # Vosk model path
        self.model = Model("/home/qaim/Downloads/vosk-model-small-en-us-0.15")
        self.rate = 16000
        self.recognizer = KaldiRecognizer(self.model, self.rate)
        self.p = pyaudio.PyAudio()
        self.stream = None

        # Recording state
        self.recording = False
        self.frames = []
        self.running = True

        # Start ROS spin thread (non-blocking)
        threading.Thread(target=self.spin_ros, daemon=True).start()

        # Start Tkinter GUI in main thread
        self.gui_thread = threading.Thread(target=self.run_gui)
        self.gui_thread.start()

        self.get_logger().info("Speech-to-text node started (Tkinter PTT GUI).")

    def spin_ros(self):
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_gui(self):
        self.root = tk.Tk()
        self.root.title("Push-to-Talk (PTT) STT Node")

        self.label = tk.Label(self.root, text="Hold the button and speak.\nRelease to send.", font=("Arial", 16))
        self.label.pack(padx=20, pady=20)

        self.button = tk.Button(self.root, text="🎤 Hold to Talk", font=("Arial", 24), width=16, height=2, bg="green")
        self.button.pack(padx=20, pady=40)

        # Bind button press/release
        self.button.bind('<ButtonPress>', lambda e: self.start_recording())
        self.button.bind('<ButtonRelease>', lambda e: self.stop_recording())

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def start_recording(self):
        if self.recording:
            return
        self.recording = True
        self.frames = []
        self.label.config(text="Listening... (Release button when done)")
        self.button.config(bg="red")
        self.get_logger().info("Mic opened (Hold-to-Talk). Recording...")

        self.stream = self.p.open(format=pyaudio.paInt16,
                                  channels=1,
                                  rate=self.rate,
                                  input=True,
                                  frames_per_buffer=4000)
        self._record_thread = threading.Thread(target=self.record_audio)
        self._record_thread.start()

    def record_audio(self):
        while self.recording:
            data = self.stream.read(4000, exception_on_overflow=False)
            self.frames.append(data)

    def stop_recording(self):
        if not self.recording:
            return
        self.recording = False
        self.label.config(text="Processing...")
        self.button.config(bg="green")
        self.get_logger().info("Mic closed. Processing speech...")
        # Wait for record thread to finish
        if hasattr(self, '_record_thread'):
            self._record_thread.join()
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None
        # Process audio
        audio_bytes = b''.join(self.frames)
        self.recognizer.Reset()
        if audio_bytes:
            for i in range(0, len(audio_bytes), 4000):
                self.recognizer.AcceptWaveform(audio_bytes[i:i+4000])
            result = json.loads(self.recognizer.FinalResult())
            text = result.get("text", "").strip()
            if text:
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
                self.label.config(text=f"Sent: {text}")
                self.get_logger().info(f"Published human speech: {text}")
            else:
                self.label.config(text="No speech detected.")
                self.get_logger().info("No speech detected.")
        else:
            self.label.config(text="No audio captured.")
            self.get_logger().info("No audio captured.")

    def on_close(self):
        self.running = False
        self.root.destroy()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    try:
        while node.running:
            pass  # All logic is in Tkinter and ROS2 threads
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
