#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
PLUGIN_DIR="$SCRIPT_DIR"
BIN_DIR="$REPO_ROOT/bin"

echo "== DrumCloud build =="
echo "Plugin dir : $PLUGIN_DIR"
echo "Repo root  : $REPO_ROOT"
echo "Bin dir    : $BIN_DIR"
echo

cd "$PLUGIN_DIR"

if [[ ! -f Makefile ]]; then
  echo "ERROR: Makefile not found in $PLUGIN_DIR"
  exit 1
fi

echo "== Cleaning previous build =="
make clean

echo
echo "== Building plugin formats =="
make CONFIG=Release

echo
echo "== Building LV2 TTL generator =="
make -C "$REPO_ROOT/utils/lv2-ttl-generator"

echo
echo "== Generating LV2 TTL files =="
(
  cd "$REPO_ROOT"
  ./utils/generate-ttl.sh
)

echo
echo "== Build results =="
ls -la "$BIN_DIR" || true

echo
echo "== Checking expected outputs =="
for item in \
  "$BIN_DIR/d_drumcloud.clap" \
  "$BIN_DIR/d_drumcloud.vst3" \
  "$BIN_DIR/d_drumcloud.lv2" \
  "$BIN_DIR/d_drumcloud-vst.so"
do
  if [[ -e "$item" ]]; then
    echo "[OK] $item"
  else
    echo "[..] Not found: $item"
  fi
done

echo
echo "Build finished."