import os, re
from datetime import datetime
from pymongo import MongoClient
import certifi

# tiny .env loader (no external deps)
def load_env_file(path: str = ".env", override: bool = False) -> None:
    if not os.path.isfile(path):
        return
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            k = k.strip()
            v = v.strip().strip("'").strip('"')
            v = re.sub(r"\$\{([^}]+)\}", lambda m: os.environ.get(m.group(1), ""), v)
            if override or k not in os.environ:
                os.environ[k] = v

load_env_file(".env")

uri = os.environ.get("MONGO_URI", "").strip()
db_name = os.environ.get("MONGO_DB", "HAR_Project")
col_name = os.environ.get("MONGO_COLLECTION", "SensorData")

if not uri:
    raise SystemExit("MONGO_URI missing. Put your Atlas SRV URI in .env")

# Use certifi CA bundle for clean TLS
client = MongoClient(uri, tlsCAFile=certifi.where())

# Quick ping (fast fail if network/TLS is wrong)
client.admin.command("ping")
print("Mongo ping OK")

col = client[db_name][col_name]
doc = {"_type":"smoke_test","ts":datetime.utcnow().isoformat()}
result = col.insert_one(doc)
print("Inserted ID:", result.inserted_id)
print("One doc:", col.find_one({"_type":"smoke_test"}))
