import sqlite3

DB_FILE = 'memory.db'

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

print("Database and table created successfully!")
