"""HTTPS front end: authentication, request routing and the audit log."""

from __future__ import annotations

import hmac
import ipaddress
import json
import os
import signal
import socket
import ssl
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable
from urllib.parse import parse_qs, urlsplit

from . import __version__
from .backend import create as create_backend
from .config import NodeConfig, url_host
from .node import Node, Rejected, Settings
from .profile import load as load_profile

MAX_BODY = 262144
MIN_TOKEN_LENGTH = 32
QUIET = {"/v1/heartbeat", "/v1/setpoint"}


def parse_users(data: Any) -> dict[str, str]:
    if not isinstance(data, dict) or any(not isinstance(k, str) or not isinstance(v, str) or len(v) < MIN_TOKEN_LENGTH
                                         for k, v in data.items()):
        raise ValueError(f"users must map names to tokens of at least {MIN_TOKEN_LENGTH} characters")
    if len(set(data.values())) != len(data):
        raise ValueError("user tokens must be unique")
    return data


class UserStore:
    """users.json maps user names to bearer tokens; edits take effect without a restart."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._lock = threading.Lock()
        self._stamp: tuple[int, int] | None = None
        self._users: dict[str, bytes] = {}
        self._load(strict=True)

    def _load(self, strict: bool) -> None:
        try:
            stat = self.path.stat()
            stamp = (stat.st_mtime_ns, stat.st_size)
            if stamp == self._stamp:
                return
            users = parse_users(json.loads(self.path.read_text(encoding="utf-8-sig")))
        except (OSError, ValueError) as exc:
            if strict:
                raise ValueError(f"cannot load users from {self.path}: {exc}") from None
            return  # keep the last valid set
        self._users = {name: token.encode() for name, token in users.items()}
        self._stamp = stamp

    def names(self) -> list[str]:
        with self._lock:
            self._load(strict=False)
            return sorted(self._users)

    def authenticate(self, token: str) -> str | None:
        with self._lock:
            self._load(strict=False)
            users = self._users
        given = token.encode("utf-8", "surrogateescape")
        match = None
        for name, expected in users.items():
            if hmac.compare_digest(given, expected):
                match = name
        return match


class Handler(BaseHTTPRequestHandler):
    server_version = f"rail-node/{__version__}"
    sys_version = ""
    server: NodeServer

    def log_message(self, format: str, *args: Any) -> None:
        pass  # requests go to the audit log, which never records tokens or lease ids

    def _reply(self, status: int, value: Any) -> None:
        raw = json.dumps(value, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def _body(self) -> dict[str, Any]:
        if self.headers.get("Transfer-Encoding"):
            raise Rejected("chunked bodies are not accepted")
        try:
            size = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            raise Rejected("invalid Content-Length") from None
        if not 0 < size <= MAX_BODY:
            raise Rejected(f"body must be 1..{MAX_BODY} bytes")
        try:
            data = json.loads(self.rfile.read(size))
        except ValueError:
            raise Rejected("body must be JSON") from None
        if not isinstance(data, dict):
            raise Rejected("body must be a JSON object")
        return data

    def _local(self) -> bool:
        try:
            return ipaddress.ip_address(self.client_address[0]).is_loopback
        except ValueError:
            return False

    def _dispatch(self) -> None:
        parts = urlsplit(self.path)
        user, status, path = None, 200, parts.path
        try:
            header = self.headers.get("Authorization", "")
            if not header.startswith("Bearer "):
                raise Rejected("bearer token required", 401)
            user = self.server.users.authenticate(header[7:].strip())
            if user is None:
                raise Rejected("invalid token", 401)
            node = self.server.node
            if self.command == "GET":
                if path == "/v1/state":
                    result = node.snapshot(fresh=parse_qs(parts.query).get("fresh") == ["1"])
                elif path == "/v1/capabilities":
                    result = node.capabilities()
                elif path == "/v1/clock":
                    result = {"server_time": node.clock()}
                else:
                    raise Rejected("unknown endpoint", 404)
            else:
                data = self._body()
                routes: dict[str, Callable[[str, dict[str, Any]], Any]] = {
                    "/v1/lease": node.acquire, "/v1/heartbeat": node.heartbeat, "/v1/release": node.release,
                    "/v1/setpoint": node.setpoint, "/v1/task": node.submit_task,
                    "/v1/stop": lambda u, _: node.stop(u), "/v1/reset": lambda u, _: node.reset(u, self._local()),
                    "/v1/park": lambda u, _: node.park(u),
                }
                if path not in routes:
                    raise Rejected("unknown endpoint", 404)
                result = routes[path](user, data)
        except Rejected as exc:
            status, result = exc.status, {"error": str(exc)}
        except Exception as exc:
            status, result = 500, {"error": f"internal error: {type(exc).__name__}: {exc}"}
        if (self.command != "GET" and path not in QUIET) or status >= 400:
            self.server.audit(user=user, client=self.client_address[0], method=self.command, path=path,
                              status=status, error=result.get("error"))
        self._reply(status, result)

    do_GET = _dispatch
    do_POST = _dispatch


class NodeServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = os.name != "nt"  # on Windows SO_REUSEADDR would let a second node share the port

    def __init__(self, address: tuple[str, int], node: Node, users: UserStore, audit_path: Path | None = None,
                 tls: ssl.SSLContext | None = None, io_timeout_s: float = 5.0) -> None:
        self.node, self.users, self.tls, self.io_timeout_s = node, users, tls, io_timeout_s
        self.profile: dict[str, Any] = {}
        self.audit_path = Path(audit_path) if audit_path else None
        self._audit_lock = threading.Lock()
        super().__init__(address, Handler)

    def server_bind(self) -> None:
        if os.name == "nt" and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()

    def finish_request(self, request: Any, client_address: Any) -> None:
        request.settimeout(self.io_timeout_s)
        if self.tls is None:
            super().finish_request(request, client_address)
            return
        try:
            connection = self.tls.wrap_socket(request, server_side=True)  # handshake in this worker thread
        except OSError:
            return
        try:
            self.RequestHandlerClass(connection, client_address, self)
        finally:
            try:
                connection.close()
            except OSError:
                pass

    def handle_error(self, request: Any, client_address: Any) -> None:
        pass  # a client that drops its connection is not a node error

    def audit(self, **entry: Any) -> None:
        if self.audit_path is None:
            return
        line = json.dumps({"time": round(time.time(), 3), **entry}, ensure_ascii=False) + "\n"
        try:
            with self._audit_lock, self.audit_path.open("a", encoding="utf-8") as stream:
                stream.write(line)
        except OSError:
            pass


def tls_context(cert: Path, key: Path) -> ssl.SSLContext:
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(cert, key)
    return context


def is_loopback(host: str) -> bool:
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def build(config: NodeConfig, allow_motion: bool = False) -> NodeServer:
    profile = load_profile(config.profile)
    settings = Settings.from_profile(profile, config.settings)
    users = UserStore(config.users)
    if config.tls_cert is None and not is_loopback(config.host):
        raise ValueError("a listener reachable from other computers requires tls_cert and tls_key")
    tls = tls_context(config.tls_cert, config.tls_key) if config.tls_cert and config.tls_key else None
    backend = create_backend(profile, allow_motion or config.allow_motion)
    mocap = None
    if "mocap" in profile:
        from .mocap import MocapReceiver

        mocap = MocapReceiver(profile["mocap"])
    node = Node(backend, settings, mocap=mocap, remote_reset=config.remote_reset)
    server = NodeServer((config.host, config.port), node, users, config.audit, tls)
    server.profile = profile
    return server


def _console_quick_edit_off() -> None:
    """A click in a Windows console with QuickEdit pauses every write to it until a key is pressed."""
    if os.name != "nt":
        return
    try:
        import ctypes

        kernel = ctypes.windll.kernel32  # type: ignore[attr-defined]
        handle = kernel.GetStdHandle(-10)
        mode = ctypes.c_uint32()
        if kernel.GetConsoleMode(handle, ctypes.byref(mode)):
            kernel.SetConsoleMode(handle, (mode.value | 0x80) & ~0x40)
    except Exception:
        pass


def serve(config: NodeConfig, allow_motion: bool = False, out: Callable[[str], None] = print) -> int:
    _console_quick_edit_off()
    server = build(config, allow_motion)
    node, backend, profile = server.node, server.node.backend, server.profile

    def event(kind: str, detail: str) -> None:
        out(f"{time.strftime('%H:%M:%S')}  {kind:<9}{detail}")
        server.audit(event=kind, detail=detail)

    node.on_event = event
    scheme = "https" if server.tls else "http"
    where = profile["modbus"]["host"] if profile["driver"] == "modbus" else "simulated rail"
    motion = backend.allow_motion
    out(f"rail-node {__version__}  {scheme}://{url_host(config.address)}:{config.port}  "
        f"(listening on {config.host}:{config.port})")
    out(f"backend   {backend.name} · {where} · {'MOTION ENABLED' if motion else 'read-only'}")
    out(f"axes      {' '.join(n + ('' if a.motion else '(read)') for n, a in backend.axes.items())}")
    if node.mocap:
        out(f"mocap     vrpn://{node.mocap.host}:{node.mocap.port} · {' '.join(node.mocap.trackers)}")
    remote = "  (faults can be reset remotely)" if node.remote_reset else ""
    out(f"users     {' '.join(server.users.names())}{remote}")
    held = [n for n, a in backend.axes.items() if a.hold]
    if held:
        parked = [n for n in held if backend.axes[n].park is not None]
        out(f"holding   {' '.join(held)} stay enabled through stops and faults"
            + (f"; {' '.join(parked)} park before they are released" if parked else ""))
    out("Ctrl+C " + ("parks the held axes, then stops the node" if held else "stops the node and every axis it moved"))
    server.audit(event="start", detail=f"{backend.name} motion={motion}")

    def interrupt(signum: int, frame: Any) -> None:
        raise KeyboardInterrupt

    for name in ("SIGTERM", "SIGBREAK"):
        if hasattr(signal, name):
            try:
                signal.signal(getattr(signal, name), interrupt)
            except ValueError:
                pass  # not in the main thread
    node.start()
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        out("stopping")
        node.close()
        server.server_close()
        server.audit(event="stop", detail=node.fault or "clean shutdown")
    return 0
