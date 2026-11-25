#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8096}"

if [ ! -d ".venv" ]; then
  uv sync --python 3.12 --extra dev
fi

exec uv run python -m motrix_robot_web --host "$HOST" --port "$PORT" "$@"
