from flask import Flask, render_template, request, redirect, url_for, session, jsonify, Response
import sqlite3
import os
import plotly
import plotly.express as px
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DB_FILE = 'memory.db'
SECRET_KEY = 'dev-secret'

# --- ROS2 PUBLISHER for User ID ---
class ROSUserPublisher(Node):
    def __init__(self):
        super().__init__('dashboard_user_id_publisher')
        self.publisher = self.create_publisher(String, 'current_user_id', 10)

    def publish_user_id(self, user_id):
        msg = String()
        msg.data = user_id
        self.publisher.publish(msg)

# Flask Setup
app = Flask(__name__)
app.secret_key = SECRET_KEY

# Ensure the database and table exist so the app can run without manual setup
def ensure_db():
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute('''
        CREATE TABLE IF NOT EXISTS memory (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            user_id TEXT,
            user_utterance TEXT,
            robot_reply TEXT,
            valence REAL,
            turn_index INTEGER,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    ''')
    conn.commit()
    conn.close()

# Create DB file/table if missing when the application starts
ensure_db()

# --- ROS2 Node Thread ---
def ros2_spin_thread():
    rclpy.init()
    node = ROSUserPublisher()
    app.ros_node = node
    rclpy.spin(node)
    rclpy.shutdown()
threading.Thread(target=ros2_spin_thread, daemon=True).start()

# --- Helper Functions ---
def get_all_user_ids():
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute("SELECT DISTINCT user_id FROM memory ORDER BY user_id ASC;")
    users = [row[0] for row in c.fetchall()]
    conn.close()
    return users

def get_latest_conversations(user_id, limit=10):
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute('''SELECT user_utterance, robot_reply, valence, category, timestamp
                 FROM memory WHERE user_id=? ORDER BY id DESC LIMIT ?;''',
              (user_id, limit))
    convs = c.fetchall()
