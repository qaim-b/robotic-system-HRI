# 🤖 Emotion-Aware and Memory-Allocated Turtlebot3 Dialogue System

## 🚀 Project Goal
This project builds a robot that can:
- **Feel emotions**: Uses a pulse sensor and HRV (pNN50) to find user mood.  
- **Remember conversations**: Saves chat in a memory database and brings back old memories.  
- **Reply smartly**: Uses Google Gemini API to give natural, empathetic replies.  
- **Talk and listen**: Captures user voice (STT) and speaks back (TTS).  
- **Show feelings**: Displays happy/sad faces on a GUI.  
- **Log data**: Dashboard shows charts, logs, and exports CSV for research.  

This project helps people, especially elderly people, in care homes by giving them meaningful and supportive conversations.  

---

## 🏗️ Project Structure
emotion_aware_robot/
├── pnnx2.py              # HRV sensor reader & pNN50 calculator
├── pnnx_to_emotion2.py   # Maps pNN50 → valence
├── speech_to_text_node.py# Vosk STT capture (Tkinter PTT GUI)
├── gemini_node.py        # Reply generation with Gemini API
├── memory_node.py        # SQLite memory manager
├── face_publisher.py     # Publishes facial expressions (/face)
├── dashboard/            # Flask + Plotly experiment dashboard
├── conversation_log.csv  # Logged conversations
├── baseline_reflection_log.csv # HRV baseline/reflection logs
├── README.md             # This file!

---

## 💻 Roadmap
1. **Emotion Recognition**  
   - Collect heart rate → calculate pNN50 → classify valence.  
   - Publishes data to ROS2 topic `/emotion_state`.  

2. **Speech Input & Output**  
   - User speech → transcribed by Vosk STT.  
   - Robot reply → spoken with gTTS.  
   - Echo prevention using `/speech_start` and `/speech_end`.  

3. **Memory Module**  
   - Saves all conversation turns in SQLite (`memory.db`).  
   - Retrieves important memories and injects them into replies.  

4. **Reply Generation (Gemini API)**  
   - Builds prompt with user speech + memories + emotion.  
   - Calls Gemini → returns empathetic, natural reply.  

5. **Dashboard**  
   - Flask + Plotly web app.  
   - Shows conversation history, emotion trends, memory usage.  
   - Export CSV logs for analysis.  

---

## 📝 Portfolio Story
I built a robot that can **sense, remember, and respond with empathy**.  
It combines emotion recognition (pNN50), memory, and Gemini AI to create meaningful conversations.  
A web dashboard shows real-time logs and analysis for research use.  

---

## 📄 Resume Highlights
- Built **ROS2 robot pipeline** with multiple nodes (sensor, memory, AI, dashboard).  
- Integrated **pNN50 HRV emotion sensing** with **Gemini LLM replies**.  
- Developed **SQLite memory retrieval** algorithm with valence weighting.  
- Created **Flask dashboard** to visualize emotion trends and export logs.  

---

## 🛠️ How to Run the Project
```bash
# Clone repo
git clone https://github.com/username/emotion_aware_robot
cd emotion_aware_robot

# Build ROS2 workspace
colcon build
source install/setup.bash

# Run nodes
ros2 run emotion_speech pnnx2.py
ros2 run emotion_speech pnnx_to_emotion2.py
ros2 run emotion_speech speech_to_text_node.py
ros2 run emotion_speech gemini_node.py
ros2 run emotion_speech memory_node.py
ros2 run emotion_speech face_publisher.py

✅ Next Steps

Add multilingual support (English + Japanese).

Improve long-term memory algorithm.

Add more dashboard analysis tools.
