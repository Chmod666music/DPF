#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
BIN_DIR="$REPO_ROOT/bin"

CLAP_DST="${HOME}/.clap"
VST3_DST="${HOME}/.vst3"
LV2_DST="${HOME}/.lv2"
VST2_DST="${HOME}/.vst"

echo "== DrumCloud install =="
echo "Bin dir: $BIN_DIR"
echo

mkdir -p "$CLAP_DST" "$VST3_DST" "$LV2_DST" "$VST2_DST"

install_item() {
  local src="$1"
  local dst="$2"

  if [[ -e "$src" ]]; then
    echo "Installing: $src -> $dst"
    cp -av "$src" "$dst/"
  else
    echo "Skipping missing: $src"
  fi
}

install_item "$BIN_DIR/d_drumcloud.clap" "$CLAP_DST"
install_item "$BIN_DIR/d_drumcloud.vst3" "$VST3_DST"
install_item "$BIN_DIR/d_drumcloud.lv2" "$LV2_DST"
install_item "$BIN_DIR/d_drumcloud-vst.so" "$VST2_DST"
install_item "$BIN_DIR/d_drumcloud.vst" "$VST2_DST"

echo
echo "Installed plugin formats:"
echo "  CLAP -> $CLAP_DST"
echo "  VST3 -> $VST3_DST"
echo "  LV2  -> $LV2_DST"
echo "  VST2 -> $VST2_DST"
echo
echo "Now rescan plugins in your DAW."