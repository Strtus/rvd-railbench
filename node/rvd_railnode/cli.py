"""rail-node: run on the computer wired to the rail; everyone else connects to it."""

from __future__ import annotations

import argparse
import ipaddress
import json
import math
import re
import secrets
import shutil
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from . import __version__
from .backend import BackendError
from .backend import create as create_backend
from .client import NodeClient, NodeError
from .config import NodeConfig, url_host
from .config import load as load_config
from .profile import SIMULATOR, ProfileError, validate
from .profile import load as load_profile

LOCAL_USER = "local"
NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,31}")


class CliError(Exception):
    pass


def _write(path: Path, text: str | bytes, private: bool = False) -> None:
    if isinstance(text, bytes):
        path.write_bytes(text)
    else:
        path.write_text(text, encoding="utf-8")
    if private:
        path.chmod(0o600)


def _identity(address: str) -> tuple[bytes, bytes, bytes, bytes, str]:
    try:
        from cryptography import x509
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import ec
        from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID
    except ImportError:
        raise CliError("rail-node init needs the cryptography package: pip install cryptography") from None
    now = datetime.now(timezone.utc)

    def name(text: str) -> x509.Name:
        return x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, text)])

    def alt(text: str) -> x509.GeneralName:
        try:
            return x509.IPAddress(ipaddress.ip_address(text))
        except ValueError:
            return x509.DNSName(text)

    ca_key = ec.generate_private_key(ec.SECP256R1())
    ca_name = name(f"rail-node CA {address}")
    ca = (x509.CertificateBuilder().subject_name(ca_name).issuer_name(ca_name).public_key(ca_key.public_key())
          .serial_number(x509.random_serial_number()).not_valid_before(now - timedelta(minutes=5))
          .not_valid_after(now + timedelta(days=3650))
          .add_extension(x509.BasicConstraints(ca=True, path_length=0), critical=True)
          .add_extension(x509.KeyUsage(digital_signature=False, content_commitment=False, key_encipherment=False,
                                       data_encipherment=False, key_agreement=False, key_cert_sign=True,
                                       crl_sign=True, encipher_only=False, decipher_only=False), critical=True)
          .add_extension(x509.SubjectKeyIdentifier.from_public_key(ca_key.public_key()), critical=False)
          .sign(ca_key, hashes.SHA256()))
    key = ec.generate_private_key(ec.SECP256R1())
    names = list(dict.fromkeys([address, "127.0.0.1", "localhost"]))
    cert = (x509.CertificateBuilder().subject_name(name(address)).issuer_name(ca_name).public_key(key.public_key())
            .serial_number(x509.random_serial_number()).not_valid_before(now - timedelta(minutes=5))
            .not_valid_after(now + timedelta(days=1825))
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .add_extension(x509.KeyUsage(digital_signature=True, content_commitment=False, key_encipherment=False,
                                         data_encipherment=False, key_agreement=False, key_cert_sign=False,
                                         crl_sign=False, encipher_only=False, decipher_only=False), critical=True)
            .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
            .add_extension(x509.SubjectAlternativeName([alt(n) for n in names]), critical=False)
            .add_extension(x509.SubjectKeyIdentifier.from_public_key(key.public_key()), critical=False)
            .add_extension(x509.AuthorityKeyIdentifier.from_issuer_public_key(ca_key.public_key()), critical=False)
            .sign(ca_key, hashes.SHA256()))

    def pem(private: Any) -> bytes:
        return private.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                     serialization.NoEncryption())

    fingerprint = ca.fingerprint(hashes.SHA256()).hex(":")
    return (ca.public_bytes(serialization.Encoding.PEM), pem(ca_key), cert.public_bytes(serialization.Encoding.PEM),
            pem(key), fingerprint)


def _check_name(name: str) -> str:
    if not NAME.fullmatch(name):
        raise CliError(f"user name {name!r} must be 1-32 letters, digits, '.', '_' or '-'")
    if name == LOCAL_USER:
        raise CliError(f"{LOCAL_USER!r} is reserved for the node computer itself")
    return name


def _grant(root: Path, name: str, token: str, url: str) -> Path:
    directory = root / "access" / name
    directory.mkdir(parents=True)
    shutil.copyfile(root / "node-ca.pem", directory / "node-ca.pem")
    _write(directory / "connection.json", json.dumps({"url": url, "token": token, "ca_file": "node-ca.pem"},
                                                     indent=2) + "\n", private=True)
    return directory


def initialize(args: argparse.Namespace) -> int:
    root = args.directory.resolve()
    if root.exists():
        raise CliError(f"{root} already exists; init never overwrites an identity")
    names = [_check_name(n) for n in args.users]
    if len(set(names)) != len(names):
        raise CliError("user names must be unique")
    if not 0 < args.port < 65536:
        raise CliError("port must be 1..65535")
    profile = load_profile(args.profile) if args.profile else validate(SIMULATOR)
    ca, ca_key, cert, key, fingerprint = _identity(args.address)
    root.mkdir(parents=True)
    _write(root / "profile.json", json.dumps(profile, indent=2) + "\n")
    _write(root / "node-ca.pem", ca)
    _write(root / "node-ca-key.pem", ca_key, private=True)
    _write(root / "node-cert.pem", cert)
    _write(root / "node-key.pem", key, private=True)
    users = {LOCAL_USER: secrets.token_urlsafe(32), **{n: secrets.token_urlsafe(32) for n in names}}
    _write(root / "users.json", json.dumps(users, indent=2) + "\n", private=True)
    _write(root / "node.json", json.dumps({
        "address": args.address, "host": "0.0.0.0", "port": args.port, "profile": "profile.json",
        "users": "users.json", "tls_cert": "node-cert.pem", "tls_key": "node-key.pem", "audit": "audit.jsonl",
        "allow_motion": False, "remote_reset": True}, indent=2) + "\n")
    _grant(root, LOCAL_USER, users[LOCAL_USER], f"https://127.0.0.1:{args.port}")
    for name in names:
        _grant(root, name, users[name], f"https://{url_host(args.address)}:{args.port}")
    print(f"node config      {root / 'node.json'}")
    print(f"hardware         {profile['driver']}")
    print(f"certificate      {fingerprint}")
    for name in names:
        print(f"give {name:<12}{root / 'access' / name}")
    print(f"start read-only  rail-node serve --config {root / 'node.json'}")
    return 0


def add_user(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    name = _check_name(args.name)
    users = json.loads(config.users.read_text(encoding="utf-8-sig"))
    if name in users or (config.directory / "access" / name).exists():
        raise CliError(f"user {name} already exists")
    users[name] = secrets.token_urlsafe(32)
    directory = _grant(config.directory, name, users[name], f"https://{url_host(config.address)}:{config.port}")
    _write(config.users, json.dumps(users, indent=2) + "\n", private=True)
    print(f"give {name} the folder {directory}")
    return 0


def remove_user(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    name = _check_name(args.name)
    users = json.loads(config.users.read_text(encoding="utf-8-sig"))
    if name not in users:
        raise CliError(f"no user {name}")
    del users[name]
    _write(config.users, json.dumps(users, indent=2) + "\n", private=True)
    shutil.rmtree(config.directory / "access" / name, ignore_errors=True)
    print(f"{name} can no longer connect")
    return 0


def probe(args: argparse.Namespace) -> int:
    profile = load_profile(args.profile)
    backend = create_backend(profile, allow_motion=False)
    report: dict[str, Any] = {"backend": backend.name}
    healthy = True
    try:
        sample = backend.read()
        report.update(device=backend.status, raw=sample.raw, position=sample.positions,
                      read_s=round(sample.duration, 4), limits={n: a.describe() for n, a in backend.axes.items()})
    except BackendError as exc:
        report.update(device=backend.status, error=str(exc))
        healthy = False
    finally:
        backend.close()
    if "mocap" in profile:
        from .mocap import MocapReceiver

        receiver = MocapReceiver(profile["mocap"]).start()
        try:
            time.sleep(args.mocap_seconds)
            report["mocap"] = state = receiver.state()
        finally:
            receiver.close()
        for tracker in state["trackers"].values():
            if "frames" in tracker:
                tracker["rate_hz"] = round(tracker["frames"] / args.mocap_seconds, 1)
        healthy = healthy and all(t["valid"] for t in state["trackers"].values())
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if healthy else 2


def _client(args: argparse.Namespace) -> NodeClient:
    if args.connection:
        return NodeClient.from_connection(args.connection)
    if args.config:
        config: NodeConfig = load_config(args.config)
        return NodeClient.from_connection(config.directory / "access" / LOCAL_USER / "connection.json")
    raise CliError("give --config (on the node computer) or --connection")


def _value(value: Any) -> str:
    return "-" if value is None or (isinstance(value, float) and not math.isfinite(value)) else f"{value:.6f}"


def status(args: argparse.Namespace) -> int:
    client = _client(args)
    caps, state = client.capabilities(), client.state()
    if args.json:
        print(json.dumps({"capabilities": caps, "state": state}, indent=2))
        return 0
    print(f"rail-node {caps['version']} · {state['backend']} · "
          f"{'hardware connected' if state['hardware_connected'] else 'hardware NOT connected'} · "
          f"{'motion enabled' if state['motion_enabled'] else 'read-only'}")
    print(f"state     {state['state']} ({state['reason']})")
    if state["fault"]:
        print(f"fault     {state['fault']}")
    control = state["control"]
    print("control   " + ("free" if not control else
                          f"{control['owner']} · {control['mode']} · {' '.join(control['axes'])}"))
    encoders = state["sensors"]["encoders"]
    age = encoders["age_s"]
    print("encoders  " + ("ok" if encoders["ok"] else f"error: {encoders['error']}")
          + (f" · {age * 1000:.0f} ms old" if age is not None else ""))
    job = state["job"]
    if job:
        print(f"task      {job['status']} · waypoint {min(job['waypoint'] + 1, job['count'])}/{job['count']}")
    print(f"{'axis':<8}{'position':>14}{'target':>14}")
    for name, axis in caps["axes"].items():
        mark = "" if axis["motion"] else "  (read only)"
        print(f"{name:<8}{_value(state['position'].get(name)):>14}{_value(state['target'].get(name)):>14} "
              f"{axis['unit']}{mark}")
    mocap = state["sensors"].get("mocap")
    if mocap:
        print(f"mocap     {mocap['source']} · " + ("connected" if mocap["connected"] else f"error: {mocap['error']}")
              + f" · {mocap['frames']} frames")
        for name, tracker in mocap["trackers"].items():
            where = tracker.get("rail") or dict(zip("xyz", tracker.get("position_raw") or []))
            frame = "rail" if "rail" in tracker else "raw"
            pose = "  ".join(f"{k} {v:.4f}" for k, v in where.items())
            print(f"  {name:<10}" + ("ok    " if tracker["valid"] else "STALE ") + f"{frame} {pose}"
                  + (f"  ({tracker['reason']})" if tracker.get("reason") else ""))
    return 0


def stop(args: argparse.Namespace) -> int:
    state = _client(args).stop()
    print(f"stopped · state {state['state']}" + (f" · fault {state['fault']}" if state["fault"] else ""))
    return 0


def reset(args: argparse.Namespace) -> int:
    state = _client(args).reset()
    print(f"state {state['state']}" + (f" · fault {state['fault']}" if state["fault"] else ""))
    return 0


def park(args: argparse.Namespace) -> int:
    client = _client(args)
    reply = client.park()
    names = reply.get("axes") or []
    if not names:
        print("nothing to park: no held axis is enabled")
        return 0
    print(f"parking {' '.join(names)}", flush=True)
    shown = -math.inf
    try:
        while True:
            state = client.state()
            job = state["job"] or {}
            running = job.get("id") == reply["task_id"] and job.get("status") == "running"
            if time.monotonic() - shown >= 1.0 or not running:
                shown = time.monotonic()
                print("  ".join(f"{n} {_value(state['position'].get(n))}" for n in names), flush=True)
            if not running:
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        client.stop()
        print("stopped; the axes keep holding where they are", file=sys.stderr)
        raise
    if job.get("id") != reply["task_id"] or job.get("status") != "completed":
        cause = job.get("reason") or state.get("fault") or state.get("reason")
        raise CliError(f"park {job.get('status', 'lost')}: {cause}; the axes keep holding where they are")
    end = time.monotonic() + 30.0
    while set(state.get("armed", ())) & set(names) and time.monotonic() < end:
        time.sleep(0.2)
        state = client.state()
    if set(state.get("armed", ())) & set(names):
        raise CliError(f"parked, but {' '.join(sorted(set(state['armed']) & set(names)))} still enabled: "
                       f"{state.get('fault') or state.get('reason')}")
    print("parked and released")
    return 0


def _assignments(items: list[str], option: str) -> dict[str, float]:
    result: dict[str, float] = {}
    for item in items:
        name, sep, text = item.partition("=")
        name = name.strip()
        try:
            value = float(text)
        except ValueError:
            value = math.nan
        if not sep or not name or not math.isfinite(value):
            raise CliError(f"{option} expects AXIS=NUMBER, not {item!r}")
        if name in result:
            raise CliError(f"{name} is given twice")
        result[name] = value
    return result


def move(args: argparse.Namespace) -> int:
    to, by = _assignments(args.to, "--to"), _assignments(args.by, "--by")
    if not to and not by:
        raise CliError("give --to AXIS=VALUE or --by AXIS=DELTA")
    if set(to) & set(by):
        raise CliError("an axis cannot take both --to and --by")
    client = _client(args)
    state = client.state(fresh=True)
    if state["sensors"]["encoders"]["error"]:
        raise CliError(f"no position feedback: {state['sensors']['encoders']['error']}")
    goal = dict(to)
    for name, delta in by.items():
        if state["position"].get(name) is None:
            raise CliError(f"the node has no axis {name}")
        goal[name] = state["position"][name] + delta
    client.acquire("task", list(goal), period_s=1.0)
    try:
        client.task([{"position": goal, "speed": args.speed}])
        shown = -math.inf
        while True:
            state = client.state()
            job = state["job"] or {}
            if time.monotonic() - shown >= 1.0 or job.get("status") != "running":
                shown = time.monotonic()
                print("  ".join(f"{n} {_value(state['position'].get(n))}" for n in goal), flush=True)
            if job.get("status") != "running":
                break
            time.sleep(0.2)
        if job.get("status") != "completed":
            cause = job.get("reason") or state.get("fault") or state.get("reason")
            raise CliError(f"move {job.get('status', 'lost')}: {cause}")
    except KeyboardInterrupt:
        client.stop()
        print("stopped", file=sys.stderr)
        raise
    finally:
        client.release()
    print("arrived")
    return 0


def serve(args: argparse.Namespace) -> int:
    from .server import serve as run

    return run(load_config(args.config), args.allow_motion)


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(prog="rail-node", description=__doc__)
    root.add_argument("--version", action="version", version=f"rail-node {__version__}")
    sub = root.add_subparsers(dest="command", required=True)
    p = sub.add_parser("init", help="create the node configuration, TLS identity and user access folders")
    p.add_argument("--directory", type=Path, default=Path("rail-node"))
    p.add_argument("--address", required=True, help="IP address or DNS name other computers use for this node")
    p.add_argument("--profile", type=Path, help="hardware profile JSON (omit for a simulated rail)")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--users", nargs="*", default=[], metavar="NAME")
    p.set_defaults(run=initialize)
    p = sub.add_parser("probe", help="read the hardware (and motion capture) once; never writes")
    p.add_argument("--profile", type=Path, required=True)
    p.add_argument("--mocap-seconds", type=float, default=2.0, help="how long to listen to motion capture")
    p.set_defaults(run=probe)
    p = sub.add_parser("serve", help="run the node")
    p.add_argument("--config", type=Path, required=True)
    p.add_argument("--allow-motion", action="store_true", help="accept motion commands (default: read-only)")
    p.set_defaults(run=serve)
    for name, text, run in (("status", "show the node state", status), ("stop", "stop every axis", stop),
                            ("reset", "clear a fault once its cause is fixed", reset),
                            ("park", "drive the held axes to their park positions, then release them", park)):
        p = sub.add_parser(name, help=text)
        p.add_argument("--config", type=Path, help="node.json of the node on this computer")
        p.add_argument("--connection", type=Path, help="connection.json of a user")
        if name == "status":
            p.add_argument("--json", action="store_true")
        p.set_defaults(run=run)
    p = sub.add_parser("move", help="move axes to a position (--to) or by an offset (--by), then release them")
    p.add_argument("--config", type=Path, help="node.json of the node on this computer")
    p.add_argument("--connection", type=Path, help="connection.json of a user")
    p.add_argument("--to", nargs="+", default=[], metavar="AXIS=VALUE")
    p.add_argument("--by", nargs="+", default=[], metavar="AXIS=DELTA")
    p.add_argument("--speed", type=float, required=True, help="speed of every moving axis (m/s or rad/s)")
    p.set_defaults(run=move)
    for name, text, run in (("add-user", "give a new person access", add_user),
                            ("remove-user", "revoke a person's access", remove_user)):
        p = sub.add_parser(name, help=text)
        p.add_argument("name")
        p.add_argument("--config", type=Path, required=True)
        p.set_defaults(run=run)
    return root


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        return args.run(args)
    except KeyboardInterrupt:
        return 130
    except (CliError, ProfileError, NodeError, BackendError, ValueError, OSError) as exc:
        print(f"rail-node: {exc}", file=sys.stderr)
        return 2
