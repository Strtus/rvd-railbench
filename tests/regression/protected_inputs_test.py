import hashlib
import json
from pathlib import Path
import sys
root = Path(sys.argv[1])
expected = json.loads((root / "tests/regression/protected_inputs.json").read_text())
for name, checksum in expected.items():
    assert hashlib.sha256((root / name).read_bytes().replace(b"\r\n", b"\n")).hexdigest() == checksum, name
print("Original controller mathematics, generated constants and Golden datasets unchanged")
