#!/bin/sh
# curl -fsSL https://raw.githubusercontent.com/Strtus/rvd-railbench/main/install.sh | sh
set -eu

REPO="${RVD_REPO:-Strtus/rvd-railbench}"
VERSION="${RVD_VERSION:-latest}"   # latest | vX.Y.Z | source
BRANCH="${RVD_BRANCH:-main}"       # branch for source builds
PREFIX="${RVD_HOME:-$HOME/.rvd-railbench}"
BIN_DIR="${RVD_BIN:-$HOME/.local/bin}"
DATA_DIR="${RVDLAB_HOME:-$HOME/.rvdlab}"

say() { printf 'rvd-railbench: %s\n' "$*"; }
die() { printf 'rvd-railbench: %s\n' "$*" >&2; exit 1; }
have() { command -v "$1" >/dev/null 2>&1; }

fetch() {
  if have curl; then curl -fsSL "$1" -o "$2"
  elif have wget; then wget -q -O "$2" "$1"
  else die "curl or wget is required"; fi
}

sha256() {
  if have sha256sum; then sha256sum "$1" | cut -d' ' -f1
  elif have shasum; then shasum -a 256 "$1" | cut -d' ' -f1
  else echo ""; fi
}

PYTHON="${RVD_PYTHON:-python3}"
have "$PYTHON" || die "Python 3.10 or newer is required"
"$PYTHON" -c 'import sys; sys.exit(sys.version_info < (3, 10))' || die "Python 3.10 or newer is required"

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT INT TERM
stage="$tmp/rvd-railbench"

asset=""
case "$(uname -s)-$(uname -m)" in
  Linux-x86_64 | Linux-amd64) asset="rvd-railbench-linux-x64.tar.gz" ;;
esac

prebuilt=0
if [ "$VERSION" != "source" ] && [ -n "$asset" ]; then
  if [ "$VERSION" = "latest" ]; then base="https://github.com/$REPO/releases/latest/download"
  else base="https://github.com/$REPO/releases/download/$VERSION"; fi
  say "downloading $base/$asset"
  if fetch "$base/$asset" "$tmp/$asset" 2>/dev/null; then
    expected=""
    if fetch "$base/SHA256SUMS" "$tmp/SHA256SUMS" 2>/dev/null; then
      expected="$(grep " $asset\$" "$tmp/SHA256SUMS" | cut -d' ' -f1 || true)"
    fi
    actual="$(sha256 "$tmp/$asset")"
    if [ -n "$expected" ] && [ -n "$actual" ] && [ "$expected" != "$actual" ]; then
      die "checksum mismatch for $asset"
    fi
    if tar -xzf "$tmp/$asset" -C "$tmp" && "$stage/build/release/rail_main" --help >/dev/null 2>&1; then
      prebuilt=1
    fi
  fi
  [ "$prebuilt" -eq 1 ] || { say "no usable prebuilt package, building from source"; rm -rf "$stage"; }
fi

if [ "$prebuilt" -eq 0 ]; then
  have git || die "git is required to build from source"
  have cmake || die "CMake 3.16 or newer and a C++17 compiler are required to build from source"
  ref="$BRANCH"
  case "$VERSION" in latest | source) ;; *) ref="$VERSION" ;; esac
  say "building $ref from source"
  git clone --quiet --depth 1 --branch "$ref" "https://github.com/$REPO.git" "$stage"
  if have ninja; then generator="Ninja"; else generator="Unix Makefiles"; fi
  cmake -S "$stage" -B "$stage/build/release" -G "$generator" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF >/dev/null \
    || die "CMake configure failed"
  cmake --build "$stage/build/release" --parallel >/dev/null || die "build failed"
  rm -rf "$stage/.git"
fi

# Profiles and experiments live in DATA_DIR, so replacing PREFIX keeps them.
rm -rf "$PREFIX.previous"
[ ! -d "$PREFIX" ] || mv "$PREFIX" "$PREFIX.previous"
mkdir -p "$(dirname "$PREFIX")"
mv "$stage" "$PREFIX"
rm -rf "$PREFIX.previous"

"$PYTHON" -m venv "$PREFIX/.venv" || die "creating a Python venv failed (Debian/Ubuntu: apt install python3-venv)"
"$PREFIX/.venv/bin/python" -m pip install --quiet --disable-pip-version-check "$PREFIX/tui" \
  || die "installing the terminal interface failed"

mkdir -p "$BIN_DIR" "$DATA_DIR"
cat > "$BIN_DIR/rvdlab" <<EOF
#!/bin/sh
: "\${RVDLAB_PROFILES:=$DATA_DIR/profiles}"
: "\${RVDLAB_EXPERIMENTS:=$DATA_DIR/experiments}"
export RVDLAB_PROFILES RVDLAB_EXPERIMENTS
exec "$PREFIX/.venv/bin/rvdlab" --root "$PREFIX" "\$@"
EOF
chmod +x "$BIN_DIR/rvdlab"

version="$(sed -n 's/^project(rvd_railbench VERSION \([0-9.]*\).*/\1/p' "$PREFIX/CMakeLists.txt")"
say "installed RVD-RailBench $version in $PREFIX ($([ "$prebuilt" -eq 1 ] && echo prebuilt || echo built from source))"
case ":$PATH:" in
  *":$BIN_DIR:"*) say "run: rvdlab" ;;
  *) say "add $BIN_DIR to PATH, then run: rvdlab" ;;
esac
