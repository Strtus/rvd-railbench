"""rail_main --bridge against a local peer that plays RVDLAB and a rail; no network beyond loopback."""
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time

rail = Path(sys.argv[1])
root = Path(sys.argv[2]).resolve()
scratch = Path(tempfile.mkdtemp(prefix="bridge-", dir=Path.cwd())).resolve()
model = root / "policies/E_PFCR-DQP_s41_step5120000"


class Peer:
    def __init__(self, stop_at_read=-1, fail_send=False):
        self.position = [18.0, 0.03, -0.02]
        self.target, self.speed = list(self.position), [0.0, 0.0, 0.0]
        self.stop_at_read, self.fail_send = stop_at_read, fail_send
        self.reads, self.sends, self.held = 0, [], False
        self.server = socket.create_server(("127.0.0.1", 0))
        self.port = self.server.getsockname()[1]
        self.last = time.monotonic()
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def advance(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        for i in range(3):
            gap = self.target[i] - self.position[i]
            step = min(abs(gap), self.speed[i] * dt)
            self.position[i] += step if gap > 0 else -step

    def answer(self, words):
        self.advance()
        if words[0] == "READ":
            self.reads += 1
            stop = int(self.reads == self.stop_at_read)
            p = " ".join(f"{v:.9f}" for v in self.position)
            return f"POS {p} {p} 0.01 0.001 {self.reads} {stop}"
        if words[0] == "SEND":
            self.sends.append(words[1:])
            if self.fail_send:
                return "ERR the node refused the setpoint"
            values = [float(w) for w in words[2:]]
            self.target, self.speed = values[:3], [abs(v) for v in values[3:]]
            return "ACK 0"
        if words[0] == "HOLD":
            self.held = True
            self.target = list(self.position)
            return "OK"
        return "ERR unknown request"

    def serve(self):
        connection, _ = self.server.accept()
        with connection, connection.makefile("rw", newline="\n") as stream:
            for line in stream:
                stream.write(self.answer(line.split()) + "\n")
                stream.flush()


def run(name, peer, steps, rc, reason, extra=()):
    directory = scratch / name
    result = subprocess.run([str(rail), "--bridge", str(peer.port), "--max-steps", str(steps), "--model-dir", model,
                             "--experiment-dir", directory, "--log", scratch / (name + ".csv"), *extra],
                            cwd=root, capture_output=True, text=True, timeout=60)
    assert result.returncode == rc, (result.returncode, result.stdout, result.stderr)
    summary = json.loads((directory / "summary.json").read_text())
    assert summary["reason"] == reason, (summary, result.stdout, result.stderr)
    metadata = json.loads((directory / "metadata.json").read_text())
    assert metadata["hardware"] is True and metadata["rail"] == "bridge", metadata
    peer.thread.join(5)
    assert peer.held, "the run must end with HOLD"
    return result


peer = Peer()
run("steps", peer, 3, 1, "MAX_STEPS")
assert len(peer.sends) == 3 and peer.reads >= 4, (peer.sends, peer.reads)
assert peer.position != [18.0, 0.03, -0.02], "commands must move the rail"

peer = Peer(stop_at_read=2)
run("stop", peer, 10, 1, "STOP_REQUESTED")

peer = Peer(fail_send=True)
result = run("send-refused", peer, 10, 1, "SEND_FAILED")
assert "rail node: the node refused the setpoint" in result.stderr, result.stderr

fast = subprocess.run([str(rail), "--bridge", "9", "--fast", "--model-dir", str(model)], cwd=root,
                      capture_output=True, text=True, timeout=30)
assert fast.returncode == 2 and "real time" in fast.stderr, (fast.returncode, fast.stderr)
print("bridge integration passed")
