"""Persistent request IDs and audit results; outstanding jobs never replay on boot."""
import json, sqlite3, time, hashlib, uuid, threading
from functools import wraps
def locked(fn):
    @wraps(fn)
    def wrapped(self,*args,**kwargs):
        with self.lock:return fn(self,*args,**kwargs)
    return wrapped

class JobStore:
    def __init__(self, path):
        self.lock = threading.RLock()
        self.db = sqlite3.connect(str(path), check_same_thread=False)
        self.db.execute('CREATE TABLE IF NOT EXISTS jobs (id TEXT PRIMARY KEY, request_id TEXT UNIQUE, signature TEXT, body TEXT)')
        self.db.commit()
        for row in self.db.execute('SELECT id,body FROM jobs').fetchall():
            job = json.loads(row[1])
            if job['status'] not in ('succeeded','failed','unknown','cancelled'):
                self.update(row[0], status='unknown', message='gateway restarted; no command was replayed')
    @locked
    def get(self, ident):
        row = self.db.execute('SELECT body FROM jobs WHERE id=?', (ident,)).fetchone()
        if not row: raise KeyError(ident)
        return json.loads(row[0])
    @locked
    def create(self, request_id, path, payload):
        signature = hashlib.sha256(json.dumps([path,payload],sort_keys=True,allow_nan=False).encode()).hexdigest()
        row = self.db.execute('SELECT id,signature FROM jobs WHERE request_id=?',(request_id,)).fetchone()
        if row:
            if row[1] != signature: raise ValueError('request ID reused with different command')
            return self.get(row[0]), False
        job = dict(id=str(uuid.uuid4()), request_id=request_id, path=path, request=payload,
                   status='queued', created_at=time.time(), updated_at=time.time(), feedback=None, result=None)
        self.db.execute('INSERT INTO jobs VALUES (?,?,?,?)',(job['id'],request_id,signature,json.dumps(job)))
        self.db.commit(); return job, True
    @locked
    def update(self, ident, **fields):
        job = self.get(ident);job.update(fields);job['updated_at'] = time.time()
        self.db.execute('UPDATE jobs SET body=? WHERE id=?',(json.dumps(job,allow_nan=False),ident));self.db.commit()
        return job
    @locked
    def recent(self):
        return [json.loads(r[0]) for r in self.db.execute('SELECT body FROM jobs ORDER BY rowid DESC LIMIT 100')]
