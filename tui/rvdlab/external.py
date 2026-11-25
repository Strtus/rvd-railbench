"""Launch the external reference runtime and relay its private TCP connection."""

from __future__ import annotations

import argparse
import configparser
import json
import math
import os
import secrets
import selectors
import socket
import subprocess
import sys
import tempfile
import threading
import time
from contextlib import ExitStack, contextmanager, suppress
from pathlib import Path
from typing import Any, Iterator

from .experiments import new_experiment_dir
from .project import Project, ProjectError, load_project
from .remote import AXES, Bridge, RemoteError, connect, remote_override

HOST = "127.0.0.1"


class ExternalBridge(Bridge):
    """Use the same node lease and measurements with an absolute setpoint deadline."""

    feedback: dict[str, Any] | None = None

    def acquire(self, period_s: float = 1.0, timeout_s: float = 1.5) -> None:
        if self.node.capabilities().get("setpoint_watchdog_deadline") is not True:
            raise RemoteError("update the rail node before using external control: it lacks setpoint watchdog support")
        super().acquire(period_s=period_s, timeout_s=timeout_s)

    def _record(self, state: dict[str, Any]) -> None:
        self.feedback = state
        super()._record(state)

    def answer(self, words: list[str]) -> str:
        if words == ["READ"]:
            self.feedback = None
            return super().answer(words)
        if words == ["HOLD"]:
            return super().answer(words)
        if len(words) != 9 or words[0] != "SEND_UNTIL":
            return "ERR unknown external bridge request"
        try:
            return self._send_until([float(word) for word in words[2:]])
        except (ValueError, RuntimeError) as error:
            self.on_fault()
            return f"ERR {error}".replace("\n", " ")

    def _send_until(self, values: list[float]) -> str:
        if self.stop_event.is_set():
            return "ACK 1"
        feedback, self.feedback = self.feedback, None
        if feedback is None:
            raise RemoteError("a fresh encoder sample is required before each setpoint")
        if not all(math.isfinite(value) for value in values) or not values[-1] > 0:
            raise RemoteError("the external bridge needs a finite setpoint and positive sample deadline")
        target = dict(zip(AXES, values[:3]))
        speeds = dict(zip(AXES, map(abs, values[3:6])))
        for axis, speed in speeds.items():
            capability = self.axes[axis]
            if speed == 0 and capability["min_speed"] > 0:
                # The node protocol has no zero-speed setpoint. Permit a
                # quantized hold only when the previous target was also reached.
                position = feedback["position"][axis]
                previous = feedback["target"].get(axis, position)
                same_count = capability["resolution"] / 4
                if max(abs(target[axis] - position), abs(previous - position)) <= same_count:
                    speeds[axis] = capability["min_speed"]
                else:
                    raise RemoteError(f"{axis} cannot use zero speed before reaching its target")
        sample = feedback["sensors"]["encoders"]["sample_time"]
        lease = self.node.lease
        if lease is None:
            return "ACK 1"
        # Applying a late command is unsafe. Give the next tick its own
        # bounded renewal window without relaxing this command's deadline.
        watchdog = feedback["server_time"] + lease["timeout_s"]
        self.node.setpoint(target, speeds, deadline=sample + values[-1], sample_time=sample,
                           watchdog_deadline=watchdog)
        return f"ACK {int(self.stop_event.is_set())}"


def _preflight(project: Project, args: argparse.Namespace) -> None:
    if project.config_error:
        raise ProjectError(f"configuration error: {project.config_error}")
    if project.build.rail_main is None:
        raise ProjectError("rail_main is not built (cmake --preset release && cmake --build --preset release)")
    if project.profile.rail.command_mode != "arrive" or project.profile.rail.compensate_latency:
        raise ProjectError("external control requires arrive mode and uncompensated encoder positions")
    if project.profile.simulation.fast or project.profile.simulation.open_loop:
        raise ProjectError("external control requires real-time closed-loop execution")
    if args.node is None:
        return
    if not args.node.is_file():
        raise ProjectError(f"connection file does not exist: {args.node}")
    if not (project.explicit_configs or project.rail_profile):
        raise ProjectError("--node requires an explicit rail configuration or an active rail profile")
    config = configparser.ConfigParser(interpolation=None, delimiters=("=",))
    config.read(project.profile.files, encoding="utf-8")
    required = {"rail": ("minimum_m", "maximum_m", "velocity_m_s", "acceleration_m_s2"),
                "mapping": ("axis", "sign", "scale", "target_m")}
    missing = [f"{section}.{key}" for section, keys in required.items() for key in keys
               if not config.has_option(section, key)]
    if missing:
        raise ProjectError("the node rail configuration must explicitly define " + ", ".join(missing))


@contextmanager
def _publish(path: Path, payload: dict[str, Any]) -> Iterator[None]:
    # Publish the complete file without replacing another run's endpoint.
    handle, temporary = tempfile.mkstemp(prefix=".rvdlab-ready-", dir=path.parent)
    pending = Path(temporary)
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            json.dump(payload, stream)
            stream.write("\n")
        if os.name == "nt":
            pending.rename(path)  # On Windows rename fails if the target exists.
        else:
            os.link(pending, path)
        identity = path.stat()
        try:
            yield
        finally:
            with suppress(FileNotFoundError):
                if os.path.samestat(path.stat(), identity):
                    path.unlink()
    finally:
        pending.unlink(missing_ok=True)


def _accept_runtime(listener: socket.socket, process: subprocess.Popen) -> socket.socket:
    limit = time.monotonic() + 5.0
    listener.settimeout(0.25)
    while time.monotonic() < limit:
        try:
            connection, _ = listener.accept()
            return connection
        except TimeoutError:
            if process.poll() is not None:
                break
    raise ProjectError("the external runtime did not connect")


def _authenticated(connection: socket.socket, token: str, limit: float) -> bool:
    expected = f"HELLO {token}".encode("ascii")
    line = bytearray()
    while len(line) <= len(expected) + 1:
        remaining = limit - time.monotonic()
        if remaining <= 0:
            return False
        connection.settimeout(remaining)
        try:
            byte = connection.recv(1)
        except OSError:
            return False
        if not byte:
            return False
        if byte == b"\n":
            return secrets.compare_digest(line.removesuffix(b"\r"), expected)
        line.extend(byte)
    return False


def _accept_client(listener: socket.socket, token: str) -> socket.socket:
    limit = time.monotonic() + 60.0
    while (remaining := limit - time.monotonic()) > 0:
        listener.settimeout(min(remaining, 0.25))
        try:
            connection, _ = listener.accept()
        except TimeoutError:
            continue
        authenticated = False
        try:
            authenticated = _authenticated(connection, token, min(limit, time.monotonic() + 2.0))
            if authenticated:
                return connection
        finally:
            if not authenticated:
                connection.close()
    raise ProjectError("no authenticated external client connected within 60 seconds")


def _send(destination: socket.socket, data: bytes) -> bool:
    try:
        destination.sendall(data)
    except OSError:
        return False
    return True


def _end_writes(connection: socket.socket) -> None:
    try:
        connection.shutdown(socket.SHUT_WR)
    except OSError:
        pass


def _relay(client: socket.socket, runtime: socket.socket) -> None:
    # Forward bytes unchanged. A client write EOF first ends runtime's input,
    # while leaving the runtime's final END available to a half-closed client.
    with selectors.DefaultSelector() as selector:
        for peer in (client, runtime):
            peer.settimeout(1.0)
            peer.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            selector.register(peer, selectors.EVENT_READ)
        client_writable = True
        while True:
            for key, _ in selector.select(timeout=0.25):
                source = key.fileobj
                try:
                    data = source.recv(4096)
                except OSError:
                    data = b""
                if source is runtime:
                    if not data:
                        _end_writes(client)
                        return
                    if client_writable:
                        client_writable = _send(client, data)
                elif not data or not _send(runtime, data):
                    selector.unregister(client)
                    _end_writes(runtime)


def _wait_runtime(process: subprocess.Popen) -> int:
    try:
        return process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        raise ProjectError("the external runtime did not stop after its connections closed") from None


def _start_runtime(project: Project, args: argparse.Namespace, controls: ExitStack,
                   runtime_port: int, experiment_dir: Path, bridge_fault: threading.Event) -> subprocess.Popen:
    command = [str(project.build.rail_main.resolve())]
    for file in project.profile.files:
        command.extend(("--config", str(file)))
    if args.node is not None:
        node = controls.enter_context(connect(args.node))
        bridge = ExternalBridge(node, log=lambda message: print(message, file=sys.stderr),
                                on_fault=bridge_fault.set)
        controls.callback(bridge.close)
        bridge.acquire(period_s=project.profile.rail.period_s)
        bridge.record(experiment_dir / "sensors.csv")
        bridge.start()
        temporary = Path(controls.enter_context(tempfile.TemporaryDirectory(prefix="rvdlab-external-")))
        override = temporary / "node.ini"
        override.write_text(remote_override(args.max_steps), encoding="utf-8")
        command.extend(("--config", str(override), "--bridge", str(bridge.port)))
    command.extend(("--external-port", str(runtime_port), "--experiment-dir", str(experiment_dir),
                    "--max-steps", str(args.max_steps)))
    if args.name is not None:
        command.extend(("--external-name", args.name))
    # The launcher handles Ctrl+C, closes the relay, and lets the runtime
    # finish its experiment and release the node through its usual hold path.
    separate_group = ({"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt"
                      else {"start_new_session": True})
    return subprocess.Popen(command, cwd=project.root, stdin=subprocess.DEVNULL, **separate_group)


def _run(project: Project, args: argparse.Namespace) -> int:
    experiment_dir = new_experiment_dir(project.experiments_dir)
    bridge_fault = threading.Event()
    with ExitStack() as controls, socket.create_server((HOST, 0), backlog=1) as user_listener:
        endpoint = {"host": HOST, "port": user_listener.getsockname()[1],
                    "period_s": project.profile.rail.period_s, "experiment_dir": str(experiment_dir)}
        token = secrets.token_urlsafe(32)
        with _publish(args.ready_file, {**endpoint, "token": token}):
            print(json.dumps(endpoint), flush=True)
            client = _accept_client(user_listener, token)
            user_listener.close()
            process = None
            try:
                with client, socket.create_server((HOST, 0), backlog=1) as runtime_listener:
                    process = _start_runtime(project, args, controls, runtime_listener.getsockname()[1],
                                             experiment_dir, bridge_fault)
                    with _accept_runtime(runtime_listener, process) as runtime:
                        runtime_listener.close()
                        _relay(client, runtime)
            finally:
                if process is not None:
                    code = _wait_runtime(process)
    return code or (2 if bridge_fault.is_set() else 0)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="rvdlab external",
                                     description="Run an experiment using an external controller on this computer.")
    parser.add_argument("--ready-file", type=Path, required=True,
                        help="new JSON file that publishes the localhost endpoint")
    parser.add_argument("--max-steps", type=int, required=True, help="positive number of control steps")
    parser.add_argument("--name", help="label for the external model in experiment metadata")
    parser.add_argument("--root", type=Path, help="RVD-RailBench installation or checkout")
    parser.add_argument("--config", action="append", type=Path, help="runtime INI file, applied in order")
    parser.add_argument("--experiments", type=Path, help="directory for experiment records")
    parser.add_argument("--profiles", type=Path, help="directory for RVDLAB rail profiles")
    parser.add_argument("--node", type=Path, help="explicitly use the rail node in this connection.json")
    args = parser.parse_args(argv)
    if not 1 <= args.max_steps <= 2**31 - 1:
        parser.error("--max-steps must be a positive 32-bit signed integer")
    if args.name is not None and (not 1 <= len(args.name.encode("utf-8")) <= 80 or
                                  any(ord(char) < 32 or ord(char) == 127 for char in args.name)):
        parser.error("--name must be 1..80 UTF-8 bytes without ASCII control characters")
    args.ready_file = args.ready_file.expanduser().absolute()
    if args.node is not None:
        args.node = args.node.expanduser().resolve()
    configs = [path.expanduser().resolve() for path in args.config] if args.config else None
    try:
        project = load_project(args.root, configs, args.experiments, args.profiles)
        _preflight(project, args)
        return _run(project, args)
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError, configparser.Error) as error:
        print(f"external: {error}", file=sys.stderr)
        return 2
