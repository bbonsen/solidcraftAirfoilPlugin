#!/usr/bin/env bash
# Build the plugin into dist/solidcraft-airfoil/ (plugin.wasm + plugin.json).
# Needs: Rust with the wasm32-unknown-unknown target, and wasm-tools
# (https://github.com/bytecodealliance/wasm-tools).
set -euo pipefail
cd "$(dirname "$0")"
cargo build --release --lib --target wasm32-unknown-unknown
out=dist/solidcraft-airfoil
rm -rf "$out"; mkdir -p "$out"
wasm-tools component new "${CARGO_TARGET_DIR:-target}/wasm32-unknown-unknown/release/solidcraft_airfoil.wasm" -o "$out/plugin.wasm"
cp plugin.json README.md LICENSE "$out/"
echo "built $out"
