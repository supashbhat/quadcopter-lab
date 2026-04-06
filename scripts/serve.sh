#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PORT="${PORT:-4173}"
python3 ./scripts/build_bundle.py
echo "Serving Aerial Control Lab at http://localhost:${PORT}"
python3 -m http.server "${PORT}"
