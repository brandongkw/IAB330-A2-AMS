import os, json, time, pathlib
from pymongo import MongoClient
import certifi

def _ensure_dir(path: str):
    p = pathlib.Path(path)
    try:
        if p.exists():
            if not p.is_dir():
                p.rename(p.with_name(p.name + f".bak_{int(time.time())}"))
        else:
            p.mkdir(parents=True, exist_ok=True)
    except FileExistsError:
        pass

class MongoSink:
    def __init__(self, uri, db, collection, backlog_dir="data/backlog"):
        self.client = MongoClient(uri, tlsCAFile=certifi.where())
        self.col = self.client[db][collection]
        self.backlog_dir = backlog_dir
        _ensure_dir(self.backlog_dir)

    def insert_batch(self, docs):
        if not docs:
            return
        try:
            self.col.insert_many(docs, ordered=False)
            # remove/ comment this line:
            # print(f"[mongo] Uploaded {len(docs)} docs to {self.col.full_name}")
        except Exception as e:
            _ensure_dir(self.backlog_dir)
            fname = f"{self.backlog_dir}/backlog_{int(time.time())}.ndjson"
            with open(fname, "a", encoding="utf-8") as f:
                for d in docs:
                    f.write(json.dumps(d, default=str) + "\n")
            print(f"[WARN] Mongo insert failed: {e}")

    def flush_backlog(self):
        _ensure_dir(self.backlog_dir)
        for p in pathlib.Path(self.backlog_dir).glob("backlog_*.ndjson"):
            try:
                with open(p, encoding="utf-8") as f:
                    docs = [json.loads(line) for line in f]
                if docs:
                    self.col.insert_many(docs, ordered=False)
                p.unlink()
            except Exception:
                pass
