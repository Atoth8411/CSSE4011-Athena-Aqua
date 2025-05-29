import json
import hashlib
from datetime import datetime
import os

BLOCKCHAIN_FILE = "blockchain.json"

def calculate_hash(index, timestamp, data, prev_hash):
    record = f"{index}{timestamp}{json.dumps(data)}{prev_hash}"
    return hashlib.sha256(record.encode()).hexdigest()

def append_block(data):
    chain = []

    # Attempt to read existing chain
    if os.path.exists(BLOCKCHAIN_FILE):
        with open(BLOCKCHAIN_FILE, 'r') as f:
            content = f.read().strip()
            if content:
                try:
                    chain = json.loads(content)
                except json.JSONDecodeError:
                    print("[BLOCKCHAIN] Corrupted JSON detected — starting fresh.")
                    chain = []
            else:
                print("[BLOCKCHAIN] blockchain.json is empty — starting fresh.")
    else:
        print("[BLOCKCHAIN] No blockchain file found — creating new one.")

    # Create new block
    index = len(chain)
    timestamp = datetime.now().isoformat()
    prev_hash = chain[-1]["hash"] if chain else "0"
    new_hash = calculate_hash(index, timestamp, data, prev_hash)

    block = {
        "index": index,
        "timestamp": timestamp,
        "data": data,
        "prev_hash": prev_hash,
        "hash": new_hash
    }

    chain.append(block)

    # Save updated chain to file
    try:
        with open(BLOCKCHAIN_FILE, 'w') as f:
            json.dump(chain, f, indent=4)
        print(f"[BLOCKCHAIN] Block #{index} added.")
    except Exception as e:
        print(f"[BLOCKCHAIN] Failed to write file: {type(e).__name__} - {e}")

def verify_chain():
    if not os.path.exists(BLOCKCHAIN_FILE):
        return False  # No blockchain file exists

    with open(BLOCKCHAIN_FILE, 'r') as f:
        chain = json.load(f)

    for i in range(1, len(chain)):
        current = chain[i]
        previous = chain[i - 1]

        # Recalculate the hash of the current block
        recalculated_hash = calculate_hash(
            current["index"],
            current["timestamp"],
            current["data"],
            current["prev_hash"]
        )

        # Check hash matches
        if current["hash"] != recalculated_hash:
            print(f"❌ Block #{i} hash mismatch.")
            return False

        # Check linkage to previous hash
        if current["prev_hash"] != previous["hash"]:
            print(f"❌ Block #{i} prev_hash does not match block #{i-1}.")
            return False

    return True  # All checks passed

