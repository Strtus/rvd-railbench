"""Client for the rail-node protocol."""

from __future__ import annotations

import json
import ssl
import threading
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any, Iterable

from .node import PROTOCOL


class NodeError(RuntimeError):
    def __init__(self, message: str, status: int | None = None) -> None:
        super().__init__(message)
        self.status = status


def load_connection(path: str | Path) -> tuple[str, str, Path | None]:
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError) as exc:
        raise NodeError(f"cannot read {path}: {exc}") from None
    if not isinstance(data, dict) or not isinstance(data.get("url"), str) or not isinstance(data.get("token"), str):
        raise NodeError(f"{path} needs url and token")
    ca = data.get("ca_file")
    return data["url"], data["token"], (path.parent / ca if isinstance(ca, str) and ca else None)


class NodeClient:
    def __init__(self, url: str, token: str, ca_file: str | Path | None = None, timeout_s: float = 3.0) -> None:
        parts = urllib.parse.urlsplit(url)
        if (parts.scheme not in ("http", "https") or not parts.hostname or parts.username or parts.password
                or parts.path not in ("", "/") or parts.query or parts.fragment):
            raise NodeError(f"node URL must look like https://host:port, not {url!r}")
        if parts.scheme == "http" and parts.hostname not in ("127.0.0.1", "localhost", "::1"):
            raise NodeError("plain http is only allowed for a node on this computer")
        if not token.strip():
            raise NodeError("empty token")
        self.url = url.rstrip("/")
        self.token = token.strip()
        self.timeout_s = timeout_s
        self._context = None
        if parts.scheme == "https":
            self._context = ssl.create_default_context(cafile=str(ca_file) if ca_file else None)
        self.lease: dict[str, Any] | None = None
        self.seq = 0
        self.heartbeat_error: Exception | None = None
        self._beat_stop = threading.Event()
        self._beat: threading.Thread | None = None

    @classmethod
    def from_connection(cls, path: str | Path, **kwargs: Any) -> NodeClient:
        url, token, ca = load_connection(path)
        return cls(url, token, ca, **kwargs)

    def request(self, method: str, path: str, body: dict[str, Any] | None = None,
                timeout: float | None = None) -> Any:
        headers = {"Authorization": f"Bearer {self.token}"}
        data = None
        if body is not None:
            data = json.dumps(body, allow_nan=False).encode()
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(self.url + path, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=timeout or self.timeout_s, context=self._context) as reply:
                return json.load(reply)
        except urllib.error.HTTPError as exc:
            try:
                message = json.load(exc).get("error", exc.reason)
            except (ValueError, OSError, AttributeError):
                message = exc.reason
            finally:
                exc.close()
            raise NodeError(str(message), exc.code) from None
        except (OSError, ValueError) as exc:
            raise NodeError(f"cannot reach the node at {self.url}: {exc}") from None

    def capabilities(self) -> dict[str, Any]:
        result = self.request("GET", "/v1/capabilities")
        if result.get("protocol") != PROTOCOL:
            raise NodeError(f"node speaks {result.get('protocol')!r}, this client needs {PROTOCOL}")
        return result

    def state(self, fresh: bool = False) -> dict[str, Any]:
        if fresh:
            return self.request("GET", "/v1/state?fresh=1", timeout=10)
        return self.request("GET", "/v1/state")

    def clock(self) -> float:
        return float(self.request("GET", "/v1/clock")["server_time"])

    def acquire(self, mode: str, axes: Iterable[str], period_s: float = 1.0, timeout_s: float | None = None,
                heartbeat_s: float = 0.5) -> dict[str, Any]:
        if self.lease:
            raise NodeError("this client already holds control")
        body: dict[str, Any] = {"mode": mode, "axes": list(axes), "period_s": period_s}
        if timeout_s is not None:
            body["timeout_s"] = timeout_s
        self.lease = self.request("POST", "/v1/lease", body, timeout=30)
        self.seq, self.heartbeat_error = 0, None
        self._beat_stop.clear()
        self._beat = threading.Thread(target=self._heartbeat, args=(heartbeat_s,), daemon=True,
                                      name="rail-node-heartbeat")
        self._beat.start()
        return self.lease

    def _heartbeat(self, interval: float) -> None:
        while not self._beat_stop.wait(interval):
            lease = self.lease
            if lease is None:
                return
            try:
                self.request("POST", "/v1/heartbeat", {"lease_id": lease["lease_id"]})
            except NodeError as exc:
                self.heartbeat_error = exc
                return

    def _require_lease(self) -> dict[str, Any]:
        if self.lease is None:
            raise NodeError("this client holds no control lease")
        if self.heartbeat_error:
            raise NodeError(f"control heartbeat failed: {self.heartbeat_error}")
        return self.lease

    def setpoint(self, position: dict[str, float], speed: float | dict[str, float], *, deadline: float | None = None,
                 sample_time: float | None = None) -> dict[str, Any]:
        lease = self._require_lease()
        if deadline is None:
            deadline = self.clock() + lease["timeout_s"]
        self.seq += 1
        body = {"lease_id": lease["lease_id"], "instance_id": lease["instance_id"], "seq": self.seq,
                "deadline": deadline, "position": position, "speed": speed}
        if sample_time is not None:
            body["sample_time"] = sample_time
        return self.request("POST", "/v1/setpoint", body, timeout=lease["timeout_s"] + 2.0)

    def task(self, points: list[dict[str, Any]]) -> dict[str, Any]:
        lease = self._require_lease()
        return self.request("POST", "/v1/task", {"lease_id": lease["lease_id"], "points": points})

    def stop(self) -> dict[str, Any]:
        try:
            return self.request("POST", "/v1/stop", {}, timeout=30)
        finally:
            self._end_lease()

    def reset(self) -> dict[str, Any]:
        return self.request("POST", "/v1/reset", {}, timeout=30)

    def park(self) -> dict[str, Any]:
        return self.request("POST", "/v1/park", {}, timeout=30)

    def release(self) -> None:
        lease = self.lease
        if lease is None:
            return
        self._end_lease()
        try:
            self.request("POST", "/v1/release", {"lease_id": lease["lease_id"]}, timeout=30)
        except NodeError as exc:
            if exc.status != 409:  # 409: the node already revoked control
                raise

    def _end_lease(self) -> None:
        self._beat_stop.set()
        beat, self._beat = self._beat, None
        if beat and beat is not threading.current_thread():
            beat.join(timeout=5)
        self.lease = None

    def close(self) -> None:
        self.release()

    def __enter__(self) -> NodeClient:
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
