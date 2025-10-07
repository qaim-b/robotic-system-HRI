from flask import Flask, render_template, request, redirect, url_for, session, jsonify, Response, send_file
import sqlite3
import os
import plotly
import plotly.graph_objs as go
import json
import threading
import collections
import rclpy
import pandas as pd 
from rclpy.node import Node
from std_msgs.msg import String
import io
import datetime

DB_FILE = os.path.join(os.path.dirname(__file__), 'memory.db')
SECRET_KEY = 'dev-secret'

CANONICAL_COLUMNS = [
    'timestamp', 'phase', 'pattern', 'user_id', 'session', 'turn_index',
    'user_utterance', 'robot_reply', 'valence', 'emotion_label',
    'used_memory', 'pnn50_before', 'pnn50_after', 'pnn50_emotion', 'category'
]

class ROSUserPublisher(Node):
    def __init__(self):
        super().__init__('dashboard_user_id_publisher')
        self.user_pub = self.create_publisher(String, 'current_user_id', 10)
        self.phase_pub = self.create_publisher(String, 'experiment_phase', 10)
        self.pattern_pub = self.create_publisher(String, 'experiment_pattern', 10)
        self.session_pub = self.create_publisher(String, 'current_session_id', 10)

        # ✅ FIX: use self.*, not node.*
        self.pause_pub   = self.create_publisher(String, 'experiment_pause', 10)
        self.restart_pub = self.create_publisher(String, 'experiment_restart', 10)

    def publish_user_id(self, user_id):
        print(f"[Flask] Publishing user ID: {user_id}")
        msg = String()
        msg.data = user_id
        for _ in range(5):
            self.user_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_phase(self, phase):
        print(f"[Flask] Publishing experiment phase: {phase}")
        msg = String()
        msg.data = phase
        for _ in range(5):
            self.phase_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_pattern(self, pattern):
        print(f"[Flask] Publishing conversation pattern: {pattern}")
        msg = String()
        msg.data = pattern
        for _ in range(5):
            self.pattern_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_session_id(self, session_id):
        print(f"[Flask] Publishing session ID: {session_id}")
        msg = String()
        msg.data = session_id
        for _ in range(5):
            self.session_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

app = Flask(__name__)
app.secret_key = SECRET_KEY

# Store current control values in config
app.config['CURRENT_PHASE'] = "Not set"
app.config['CURRENT_PATTERN'] = "Not set"
app.config['CURRENT_USER_ID'] = "Not set"
app.config['CURRENT_SESSION_ID'] = "Not set"

def ensure_db():
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute(
        '''CREATE TABLE IF NOT EXISTS memory (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            phase TEXT,
            pattern TEXT,
            user_id TEXT,
            session TEXT,
            turn_index INTEGER,
            user_utterance TEXT,
            robot_reply TEXT,
            valence REAL,
            emotion_label TEXT,
            used_memory TEXT,
            pnn50_before REAL,
            pnn50_after REAL,
            pnn50_emotion TEXT,
            category TEXT
        )'''
    )
    c.execute('PRAGMA table_info(memory);')
    cols = [row[1] for row in c.fetchall()]
    if 'category' not in cols:
        c.execute('ALTER TABLE memory ADD COLUMN category TEXT;')
    conn.commit()
    conn.close()

ensure_db()

def ros2_spin_thread():
    rclpy.init()
    node = ROSUserPublisher()
    app.ros_node = node
    rclpy.spin(node)
    rclpy.shutdown()

threading.Thread(target=ros2_spin_thread, daemon=True).start()

def get_all_user_ids():
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute("SELECT DISTINCT user_id FROM memory ORDER BY user_id ASC;")
    users = [row[0] for row in c.fetchall()]
    conn.close()
    return users

def get_latest_conversations(user_id, limit=30):
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute('''SELECT user_utterance, robot_reply, valence, category, turn_index, timestamp, phase, pattern
                 FROM memory WHERE user_id=? ORDER BY id DESC LIMIT ?;''',
              (user_id, limit))
    convs = c.fetchall()
    conn.close()
    return convs

def extract_top_topics(conversations, top_n=5):
    utterances = [row[0] for row in conversations]   # Always user_utterance
    all_words = " ".join(utterances).lower().split()
    stopwords = set([
        'the','a','an','is','are','i','you','and','to','it','in','of','for','on','my','your',
        'was','but','with','this','that','at','by','we','as','have','be','not','or','from'
    ])
    keywords = [w for w in all_words if w.isalpha() and w not in stopwords]
    counter = collections.Counter(keywords)
    topics = counter.most_common(top_n)
    labels = [t[0] for t in topics]
    counts = [t[1] for t in topics]
    return labels, counts

@app.route("/login", methods=["GET", "POST"])
def login():
    error = None
    users = get_all_user_ids()
    if request.method == "POST":
        user_id = request.form.get("user_id") or request.form.get("existing_user")
        user_id = user_id.strip() if user_id else None
        if user_id:
            session["user_id"] = user_id
            if hasattr(app, "ros_node"):
                app.ros_node.publish_user_id(user_id)
            else:
                print("[Flask] Warning: ROS node not ready, cannot publish user_id!")
            return redirect(url_for('dashboard'))
        else:
            error = "Please enter or select a user ID."
    return render_template("login.html", error=error, users=users)

@app.route("/logout")
def logout():
    session.clear()
    return redirect(url_for('dashboard'))

@app.route("/")
@app.route("/dashboard")
def dashboard():
    users = get_all_user_ids()
    user_id = session.get('user_id')
    conversations = []
    num_turns = 0
    avg_valence = 0
    topic_chart_json = '{}'
    # --- ADDED FOR CHART.JS SUPPORT ---
    chart_topic_labels, chart_topic_counts, emotion_time_labels, emotion_time_vals = [], [], [], []
    if user_id:
        conversations = get_latest_conversations(user_id, 30)
        num_turns = len(conversations)
        valences = [c[2] for c in conversations if c[2] is not None]
        avg_valence = round(sum(valences)/len(valences), 3) if valences else 0
        topic_labels, topic_counts = extract_top_topics(conversations, top_n=5)
        topic_data = [go.Bar(x=topic_labels, y=topic_counts, marker_color='#38bdf8')]
        topic_layout = go.Layout(
            xaxis=dict(title="Topic", tickangle=-20),
            yaxis=dict(title="Count"),
            title="Most Frequent User Topics",
            plot_bgcolor="#f8fafc",
            paper_bgcolor="#f8fafc",
            height=280,
            margin=dict(t=35, l=45, r=20, b=60)
        )
        topic_chart_json = json.dumps({"data": [d for d in topic_data], "layout": topic_layout}, cls=plotly.utils.PlotlyJSONEncoder)
        # --- CHART.JS DATA ---
        chart_topic_labels = topic_labels
        chart_topic_counts = topic_counts
        # emotion_time_labels = list of timestamps (reversed for oldest to newest)
        emotion_time_labels = [c[5] for c in conversations][::-1] if conversations else []
        # emotion_time_vals = list of valence values (reversed)
        emotion_time_vals = [c[2] for c in conversations][::-1] if conversations else []
    return render_template(
        "dashboard.html",
        users=users,
        user_id=user_id,
        conversations=conversations,
        num_turns=num_turns,
        avg_valence=avg_valence,
        topic_chart_json=topic_chart_json,
        # --- NEW FOR CHART.JS ---
        chart_topic_labels=chart_topic_labels,
        chart_topic_counts=chart_topic_counts,
        emotion_time_labels=emotion_time_labels,
        emotion_time_vals=emotion_time_vals
    )

@app.route("/experiment_control", methods=["GET", "POST"])
def experiment_control():
    PHASES = [
        "Baseline 1", "Pattern 1", "Reflection 1",
        "Baseline 2", "Pattern 2", "Reflection 2",
        "Baseline 3", "Pattern 3", "Reflection 3",
        "Baseline 4", "Pattern 4", "Reflection 4"
    ]
    PATTERNS = [
        "Memory + Emotion", "Emotion Only", "Memory Only", "Vanilla"
    ]
    user_id_options = get_all_user_ids()
    if request.method == "POST":
        user_id = request.form.get('user_id')
        if user_id == "__new__":
            user_id = request.form.get('new_user_id', '').strip()
        session_id = request.form.get('session_select')
        if session_id == "__custom__":
            session_id = request.form.get('custom_session', '').strip()
        phase = request.form['phase']
        pattern = request.form['pattern']
        app.config['CURRENT_PHASE'] = phase
        app.config['CURRENT_PATTERN'] = pattern
        app.config['CURRENT_USER_ID'] = user_id
        app.config['CURRENT_SESSION_ID'] = session_id
        if hasattr(app, "ros_node"):
            app.ros_node.publish_phase(phase)
            app.ros_node.publish_pattern(pattern)
            app.ros_node.publish_user_id(user_id)
            app.ros_node.publish_session_id(session_id)
        else:
            print("[Flask] Warning: ROS node not ready, cannot publish experiment control topics!")
        return redirect(url_for('experiment_control'))
    return render_template(
        "experiment_control.html",
        current_phase=app.config.get('CURRENT_PHASE', ''),
        current_pattern=app.config.get('CURRENT_PATTERN', ''),
        current_user_id=app.config.get('CURRENT_USER_ID', ''),
        current_session_id=app.config.get('CURRENT_SESSION_ID', ''),
        phase_options=PHASES,
        pattern_options=PATTERNS,
        user_id_options=user_id_options
    )
    
@app.route("/set_phase", methods=["POST"])
def set_phase():
    data = request.get_json()
    phase = data.get('phase')
    user_id = data.get('user_id')
    session_id = data.get('session')
    pattern = data.get('pattern')

    # Update the current config for easy reference
    app.config['CURRENT_PHASE'] = phase
    app.config['CURRENT_USER_ID'] = user_id
    app.config['CURRENT_SESSION_ID'] = session_id
    app.config['CURRENT_PATTERN'] = pattern

    # Publish to ROS topics (reuse your existing ROSUserPublisher node)
    if hasattr(app, "ros_node"):
        app.ros_node.publish_phase(phase)
        app.ros_node.publish_pattern(pattern)
        app.ros_node.publish_user_id(user_id)
        app.ros_node.publish_session_id(session_id)
        print(f"[EXPERIMENT CONTROL] Published phase={phase}, user_id={user_id}, session={session_id}, pattern={pattern}")
    else:
        print("[Flask] Warning: ROS node not ready, cannot publish experiment control topics!")

    return jsonify({"status": "ok"})

@app.route("/control", methods=["POST"])
def control():
    data    = request.get_json(force=True) or {}
    action  = (data.get("action") or "").lower()      # 'pause' | 'resume' | 'restart'
    label   = data.get("label") or ""                 # 'Baseline' | 'Pattern' | 'Reflection'
    cycle   = str(data.get("cycle") or "")
    user_id = data.get("user_id") or ""
    session = str(data.get("session") or "")
    pattern = data.get("pattern") or ""

    app.logger.info(f"[Flask] Control: {action.upper()} {label} {cycle} | user={user_id} session={session} pattern={pattern}")

    if hasattr(app, "ros_node"):
        if action in ("pause", "resume"):
            app.ros_node.pause_pub.publish(String(data=f"{action}:{label}:{cycle}"))
            # optional meta re-broadcast (keeps CSV/DB writers in-sync)
            app.ros_node.user_pub.publish(String(data=user_id))
            app.ros_node.session_pub.publish(String(data=session))
            app.ros_node.pattern_pub.publish(String(data=pattern))

        elif action == "restart":
            app.ros_node.restart_pub.publish(String(data=f"restart:{label}:{cycle}"))
            # re-broadcast phase + meta so all subscribers reset with correct context
            app.ros_node.phase_pub.publish(String(data=f"{label} {cycle}"))
            app.ros_node.user_pub.publish(String(data=user_id))
            app.ros_node.session_pub.publish(String(data=session))
            app.ros_node.pattern_pub.publish(String(data=pattern))
    else:
        app.logger.warning("[Flask] ROS node not ready; control message not published")

    return {"ok": True}, 200

@app.route("/memories")
def memories():
    user_id = request.args.get('user_id', session.get('user_id'))
    if user_id:
        memories = get_latest_conversations(user_id, 200)
    else:
        memories = []
    from collections import defaultdict
    pattern_groups = defaultdict(list)
    for row in memories:
        pattern = row[7] if len(row) > 7 and row[7] else "Unknown Pattern"
        pattern_groups[pattern].append(row)
    return render_template("memories.html", pattern_groups=pattern_groups, user_id=user_id)

@app.route("/conversations")
def conversations():
    user_id = request.args.get('user_id', session.get('user_id'))
    conversations_data = []
    if user_id:
        conversations_data = get_latest_conversations(user_id, 20)
    return render_template("conversations.html", conversations=conversations_data, user_id=user_id)

@app.route("/charts")
def charts():
    users = get_all_user_ids()
    user_id = session.get('user_id')
    conversations = []
    topic_chart_json = '{}'
    valence_chart_json = '{}'   # ensure this exists for charts.html

    if user_id:
        conversations = get_latest_conversations(user_id, 30)

        # --- Topic Chart ---
        topic_labels, topic_counts = extract_top_topics(conversations, top_n=5)
        topic_data = [go.Bar(x=topic_labels, y=topic_counts, marker_color='#38bdf8')]
        topic_layout = go.Layout(
            xaxis=dict(title="Topic", tickangle=-20),
            yaxis=dict(title="Count"),
            title="Most Frequent User Topics",
            plot_bgcolor="#f8fafc",
            paper_bgcolor="#f8fafc",
            height=280,
            margin=dict(t=35, l=45, r=20, b=60)
        )
        topic_chart_json = json.dumps(
            {"data": [d for d in topic_data], "layout": topic_layout},
            cls=plotly.utils.PlotlyJSONEncoder
        )

        # --- Valence Chart ---
        # timestamp (idx 5) and valence (idx 2), reversed for oldest→newest
        valence_times = [row[5] for row in conversations][::-1]
        valence_values = [row[2] for row in conversations][::-1]
        valence_data = [go.Scatter(x=valence_times, y=valence_values, mode='lines+markers', marker_color='#10b981')]
        valence_layout = go.Layout(
            xaxis=dict(title="Turn / Time", tickangle=-20),
            yaxis=dict(title="Valence (0–1)"),
            title="Valence Over Time",
            plot_bgcolor="#f8fafc",
            paper_bgcolor="#f8fafc",
            height=280,
            margin=dict(t=35, l=45, r=20, b=60)
        )
        valence_chart_json = json.dumps(
            {"data": valence_data, "layout": valence_layout},
            cls=plotly.utils.PlotlyJSONEncoder
        )

    # For charts.html histogram: pass compact tuples (u, r, v, t)
    chart_convs = [(row[0], row[1], row[2], row[5]) for row in conversations]

    return render_template(
        "charts.html",
        users=users,
        user_id=user_id,
        conversations=chart_convs,
        topic_chart_json=topic_chart_json,
        valence_chart_json=valence_chart_json
    )

@app.route("/baseline_reflection")
def baseline_reflection():
    csv_path = "/home/qaim/Desktop/ros2_ws/baseline_reflection_log.csv"
    try:
        df = pd.read_csv(
            csv_path,
            names=["timestamp", "phase", "pattern", "user_id", "session", "pnn50_value"]
        )
    except Exception as e:
        df = pd.DataFrame(columns=["timestamp", "phase", "pattern", "user_id", "session", "pnn50_value"])
        print(f"[DASHBOARD] Could not read baseline_reflection_log.csv: {e}")

    phases = sorted(df['phase'].dropna().unique()) if 'phase' in df.columns else []
    users = sorted(df['user_id'].dropna().unique()) if 'user_id' in df.columns else []
    patterns = sorted(df['pattern'].dropna().unique()) if 'pattern' in df.columns else []

    selected_phase = request.args.get("phase")
    selected_user = request.args.get("user_id")
    selected_pattern = request.args.get("pattern")

    df_plot = df.copy()
    if selected_phase and 'phase' in df_plot.columns:
        df_plot = df_plot[df_plot['phase'] == selected_phase]
    if selected_user and 'user_id' in df_plot.columns:
        df_plot = df_plot[df_plot['user_id'] == selected_user]
    if selected_pattern and 'pattern' in df_plot.columns:
        df_plot = df_plot[df_plot['pattern'] == selected_pattern]

    chart_json = "{}"
    if not df_plot.empty and 'timestamp' in df_plot.columns and 'pnn50_value' in df_plot.columns:
        df_plot['timestamp'] = pd.to_datetime(df_plot['timestamp'])
        df_plot = df_plot.sort_values("timestamp")
        chart = [
            go.Scatter(
                x=df_plot['timestamp'],
                y=df_plot['pnn50_value'],
                mode='lines+markers',
                name='pNN50 Value'
            )
        ]
        layout = go.Layout(
            title="Baseline & Reflection Data Over Time",
            xaxis=dict(title="Time"),
            yaxis=dict(title="pNN50 Value"),
            height=350,
            plot_bgcolor="#f8fafc",
            paper_bgcolor="#f8fafc"
        )
        chart_json = json.dumps({"data": chart, "layout": layout}, cls=plotly.utils.PlotlyJSONEncoder)

    return render_template(
        "baseline_reflection.html",
        chart_json=chart_json,
        phases=phases,
        users=users,
        patterns=patterns,
        selected_phase=selected_phase,
        selected_user=selected_user,
        selected_pattern=selected_pattern
    )

@app.route("/download_baseline_reflection_log")
def download_baseline_reflection_log():
    import io
    # Use the correct directory for your log file:
    csv_path = "/home/qaim/Desktop/ros2_ws/baseline_reflection_log.csv"
    try:
        with open(csv_path, "r") as f:
            data = f.read()
        buf = io.BytesIO(data.encode())
        return send_file(
            buf,
            mimetype='text/csv',
            as_attachment=True,
            download_name="baseline_reflection_log.csv"
        )
    except Exception as e:
        return f"Error: {e}", 500


@app.route("/memory_stats", methods=["GET"])
def memory_stats():
    csv_path = "/home/qaim/Desktop/ros2_ws/conversation_log.csv"  # [FIXED PATH]
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        df = pd.DataFrame()
        print(f"[DASHBOARD] Could not read conversation_log.csv: {e}")

    phases = sorted(df['phase'].dropna().unique()) if 'phase' in df.columns else []
    users = sorted(df['user_id'].dropna().unique()) if 'user_id' in df.columns else []
    if 'session' in df.columns:
        sessions = sorted([s for s in df['session'].dropna().unique() if s and str(s).lower() != "not set"])
    else:
        sessions = []

    selected_phase = request.args.get("phase")
    selected_user = request.args.get("user_id")
    selected_session = request.args.get("session")

    df_filtered = df.copy()
    if selected_phase and 'phase' in df_filtered.columns:
        df_filtered = df_filtered[df_filtered['phase'] == selected_phase]
    if selected_user and 'user_id' in df_filtered.columns:
        df_filtered = df_filtered[df_filtered['user_id'] == selected_user]
    if selected_session and 'session' in df_filtered.columns:
        df_filtered = df_filtered[df_filtered['session'] == selected_session]

    if "download" in request.args:
        df_export = df_filtered.rename(columns={
            'timestamp': 'Timestamp',
            'phase': 'Phase',
            'pattern': 'Pattern',
            'user_id': 'User ID',
            'session': 'Session',
            'valence': 'Valence',
            'emotion_label': 'Emotion Label',
            'used_memory': 'Used Memory',
            'category': 'Category'
        })
        columns_wanted = [
            'Timestamp', 'Phase', 'Pattern', 'User ID', 'Session', 'Turn Index',
            'User Utterance', 'Robot Reply', 'Valence', 'Emotion Label',
            'Used Memory', 'pNN50 Before', 'pNN50 After', 'pNN50 Emotion', 'Category'
        ] 
        columns_wanted = [c for c in columns_wanted if c in df_export.columns]
        df_export = df_export[columns_wanted]
        now = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        fname = f"conversation_{selected_user or 'all'}_{selected_phase or 'all'}_{selected_session or 'all'}_{now}.csv"
        buf = io.StringIO()
        df_export.to_csv(buf, index=False)
        buf.seek(0)
        return send_file(
            io.BytesIO(buf.getvalue().encode()),
            mimetype='text/csv',
            as_attachment=True,
            download_name=fname
        )

    chart_json = "{}"
    if not df_filtered.empty and 'timestamp' in df_filtered.columns and 'valence' in df_filtered.columns:
        df_filtered['timestamp'] = pd.to_datetime(df_filtered['timestamp'])
        df_filtered = df_filtered.sort_values("timestamp")
        chart = [
            go.Scatter(
                x=df_filtered['timestamp'],
                y=df_filtered['valence'],
                mode='lines+markers',
                name='Valence'
            )
        ]
        layout = go.Layout(
            title="Valence Over Time",
            xaxis=dict(title="Time"),
            yaxis=dict(title="Valence"),
            height=350,
            plot_bgcolor="#f8fafc",
            paper_bgcolor="#f8fafc"
        )
        chart_json = json.dumps({"data": chart, "layout": layout}, cls=plotly.utils.PlotlyJSONEncoder)

    preview_df = df_filtered.head(5) if not df_filtered.empty else pd.DataFrame()

    return render_template(
        "memory_stats.html",
        chart_json=chart_json,
        phases=phases,
        users=users,
        sessions=sessions,
        selected_phase=selected_phase,
        selected_user=selected_user,
        selected_session=selected_session,
        preview_df=preview_df
    )

@app.route("/experiment_pattern")
def experiment_pattern():
    db_path = os.path.join(os.path.dirname(__file__), "memory.db")
    csv_path = "/home/qaim/Desktop/ros2_ws/conversation_log.csv"  # [FIXED PATH]
    print("DB path:", db_path, "exists?", os.path.exists(db_path))
    print("CSV path:", csv_path, "exists?", os.path.exists(csv_path))

    df = None
    source = None

    # Try DB first
    if os.path.exists(db_path):
        try:
            conn = sqlite3.connect(db_path)
            df = pd.read_sql_query("SELECT * FROM memory", conn)
            conn.close()
            source = "db"
        except Exception as e:
            print("DB error:", e)

    # Fallback: Try CSV if DB fails or is empty
    if (df is None or df.empty) and os.path.exists(csv_path):
        try:
            df = pd.read_csv(csv_path)
            source = "csv"
        except Exception as e:
            print("CSV error:", e)

    if df is None or df.empty:
        return "<h3 style='color:red;'>No experiment data found in DB or CSV!</h3>"

    users = sorted(df['user_id'].dropna().unique()) if 'user_id' in df.columns else []
    if 'session' in df.columns:
        sessions = sorted([s for s in df['session'].dropna().unique() if s and str(s).lower() != "not set"])
    else:
        sessions = []
    phases = sorted(df['phase'].dropna().unique()) if 'phase' in df.columns else []

    print("DF columns:", df.columns)
    print("First 5 rows:\n", df.head())
    if 'session' in df.columns:
        print("All unique session values:", df['session'].unique())

    return render_template('experiment_pattern.html', users=users, sessions=sessions, phases=phases, source=source)

@app.route("/download_full_experiment")
def download_full_experiment():
    csv_path = "/home/qaim/Desktop/ros2_ws/conversation_log.csv"
    df = pd.read_csv(csv_path)
    user_id = request.args.get("user_id")
    session = request.args.get("session")
    if user_id:
        df = df[df['user_id'] == user_id]
    if session and 'session' in df.columns:
        df = df[df['session'] == session]
    now = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    fname = f"{user_id or 'all'}_{session or 'all'}_full_experiment_{now}.csv"
    buf = io.StringIO()
    df.to_csv(buf, index=False)
    buf.seek(0)
    return send_file(
        io.BytesIO(buf.getvalue().encode()),
        mimetype='text/csv',
        as_attachment=True,
        download_name=fname
    )

@app.route("/download_phase")
def download_phase():
    csv_path = "/home/qaim/Desktop/ros2_ws/conversation_log.csv"  # [FIXED PATH]
    df = pd.read_csv(csv_path)

    user_id = request.args.get("user_id")
    session = request.args.get("session")
    phase = request.args.get("phase")

    # Filter by user_id if present
    if user_id and 'user_id' in df.columns:
        df = df[df['user_id'] == user_id]

    # Filter by session if present and the column exists
    if session and 'session' in df.columns:
        df = df[df['session'] == session]

    # Filter by phase if present and the column exists
    if phase:
        if 'phase' in df.columns:
            df = df[df['phase'] == phase]
        else:
            return (
                f"Error: Column 'phase' does not exist in the CSV.",
                400
            )

    now = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    fname = f"{user_id or 'all'}_{session or 'all'}_{phase or 'all'}_{now}.csv"

    buf = io.StringIO()
    df.to_csv(buf, index=False)
    buf.seek(0)

    return send_file(
        io.BytesIO(buf.getvalue().encode()),
        mimetype='text/csv',
        as_attachment=True,
        download_name=fname
    )


if __name__ == "__main__":
    app.run(debug=True)
