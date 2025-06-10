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
    conn.close()
    return list(reversed(convs))

def get_dashboard_stats(user_id):
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute('SELECT COUNT(*), AVG(valence) FROM memory WHERE user_id=?;', (user_id,))
    row = c.fetchone()
    num_turns = row[0]
    avg_valence = round(row[1], 2) if row[1] is not None else 0
    c.execute('SELECT user_utterance FROM memory WHERE user_id=?;', (user_id,))
    utterances = [u[0] for u in c.fetchall()]
    topics = []
    for text in utterances:
        topics.extend([w.strip('.,?!').lower() for w in text.split() if len(w) > 3])
    from collections import Counter
    topic_counts = Counter(topics)
    conn.close()
    return num_turns, avg_valence, topic_counts.most_common(8)

@app.route('/', methods=['GET', 'POST'])
def login():
    users = get_all_user_ids()
    if request.method == 'POST':
        user_id = request.form.get('user_id') or request.form.get('user_id_new')
        if not user_id or user_id.strip() == "":
            return render_template('login.html', users=users, error='Please enter or select a user.')
        user_id = user_id.strip()
        session['user_id'] = user_id
        try:
            app.ros_node.publish_user_id(user_id)
        except Exception:
            pass
        return redirect(url_for('dashboard'))
    return render_template('login.html', users=users, error=None)

def make_topic_chart(top_topics):
    topic_labels = [x[0] for x in top_topics] if top_topics else []
    topic_values = [x[1] for x in top_topics] if top_topics else []
    if not topic_labels or not topic_values:
        topic_labels = ["No Data"]
        topic_values = [0]
    bar_fig = px.bar(x=topic_labels, y=topic_values, labels={'x':'Topic','y':'Count'},
                     title='Top Topics in Conversation')
    return json.dumps(bar_fig, cls=plotly.utils.PlotlyJSONEncoder)

@app.route('/dashboard')
def dashboard():
    user_id = session.get('user_id')
    if not user_id:
        return redirect(url_for('login'))
    num_turns, avg_valence, top_topics = get_dashboard_stats(user_id)
    conversations = get_latest_conversations(user_id)
    topic_chart_json = make_topic_chart(top_topics)
    return render_template('dashboard.html',
                           user_id=user_id,
                           num_turns=num_turns,
                           avg_valence=avg_valence,
                           conversations=conversations,
                           topic_chart_json=topic_chart_json)

@app.route('/dashboard_viewonly')
def dashboard_viewonly():
    user_id = "default_user"
    num_turns, avg_valence, top_topics = get_dashboard_stats(user_id)
    conversations = get_latest_conversations(user_id)
    topic_chart_json = make_topic_chart(top_topics)
    return render_template('dashboard.html',
                           user_id="(Observer Mode)",
                           num_turns=num_turns,
                           avg_valence=avg_valence,
                           conversations=conversations,
                           topic_chart_json=topic_chart_json)

@app.route('/conversations')
def conversations():
    user_id = session.get('user_id', 'default_user')
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute("SELECT user_utterance, robot_reply, valence, category, timestamp FROM memory WHERE user_id=? ORDER BY id DESC LIMIT 50;", (user_id,))
    conversations = c.fetchall()
    conn.close()
    return render_template('conversations.html', conversations=conversations)

@app.route('/charts')
def charts():
    user_id = session.get('user_id', 'default_user')
    num_turns, avg_valence, top_topics = get_dashboard_stats(user_id)
    topic_chart_json = make_topic_chart(top_topics)
    return render_template('charts.html',
                           num_turns=num_turns,
                           avg_valence=avg_valence,
                           top_topics=top_topics,
                           topic_chart_json=topic_chart_json)

@app.route('/memories')
def memories():
    user_id = session.get('user_id', 'default_user')
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute("SELECT user_utterance, robot_reply, valence, category, turn_index, timestamp FROM memory WHERE user_id=? ORDER BY id DESC;", (user_id,))
    memories = c.fetchall()
    conn.close()
    return render_template('memories.html', memories=memories)

@app.route('/download_csv')
def download_csv():
    user_id = session.get('user_id', 'default_user')
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute("SELECT * FROM memory WHERE user_id=? ORDER BY id DESC;", (user_id,))
    rows = c.fetchall()
    # Get header from schema
    c.execute("PRAGMA table_info(memory);")
    header = [col[1] for col in c.fetchall()]
    conn.close()
    def generate():
        yield ','.join(header) + '\n'
        for row in rows:
            yield ','.join(map(str, row)) + '\n'
    return Response(generate(), mimetype='text/csv',
                    headers={'Content-Disposition': 'attachment; filename=memories.csv'})

@app.route('/logout')
def logout():
    session.pop('user_id', None)
    return redirect(url_for('login'))

@app.route('/api/conversations')
def api_conversations():
    user_id = session.get('user_id', 'default_user')
    convs = get_latest_conversations(user_id)
    return jsonify(convs)

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')

