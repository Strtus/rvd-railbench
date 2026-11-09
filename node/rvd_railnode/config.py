"""node.json: where a node finds its profile, users and TLS identity."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class NodeConfig:
    path: Path
    address: str
    host: str
    port: int
    profile: Path
    users: Path
    tls_cert: Path | None
    tls_key: Path | None
    audit: Path | None
    allow_motion: bool
    settings: dict[str, Any]
    remote_reset: bool = False

    @property
    def directory(self) -> Path:
        return self.path.parent


def load(path: str | Path) -> NodeConfig:
    path = Path(path).resolve()
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError) as exc:
        raise ValueError(f"cannot read {path}: {exc}") from None
    if not isinstance(data, dict):
        raise ValueError(f"{path} must hold a JSON object")

    def file(key: str, required: bool = True) -> Path | None:
        value = data.get(key)
        if value is None and not required:
            return None
        if not isinstance(value, str) or not value:
            raise ValueError(f"{path}: {key} must name a file")
        return (path.parent / value).resolve()

    port = data.get("port", 8765)
    if type(port) is not int or not 0 < port < 65536:
        raise ValueError(f"{path}: port must be 1..65535")
    for key in ("address", "host"):
        if not isinstance(data.get(key, ""), str):
            raise ValueError(f"{path}: {key} must be a string")
    for key in ("allow_motion", "remote_reset"):
        if not isinstance(data.get(key, False), bool):
            raise ValueError(f"{path}: {key} must be true or false")
    if not isinstance(data.get("settings", {}), dict):
        raise ValueError(f"{path}: settings must be an object")
    tls_cert, tls_key = file("tls_cert", False), file("tls_key", False)
    if (tls_cert is None) != (tls_key is None):
        raise ValueError(f"{path}: tls_cert and tls_key go together")
    return NodeConfig(path, data.get("address") or "127.0.0.1", data.get("host") or "127.0.0.1", port,
                      file("profile"), file("users"), tls_cert, tls_key, file("audit", False),
                      data.get("allow_motion", False), dict(data.get("settings", {})), data.get("remote_reset", False))


def url_host(address: str) -> str:
    return f"[{address}]" if ":" in address and not address.startswith("[") else address
