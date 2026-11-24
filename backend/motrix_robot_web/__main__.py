from __future__ import annotations

import argparse
from pathlib import Path

import uvicorn

from .app import create_app


def main() -> None:
    parser = argparse.ArgumentParser(description="Motphys Robot Control & Perception Web Server")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8096, help="Bind port (default: 8096)")
    parser.add_argument("--config", type=Path, default=None, help="Optional YAML robot connectors config")
    parser.add_argument("--data-dir", type=Path, default=None, help="Persistent JSON store directory")
    args = parser.parse_args()

    application = create_app(data_dir=args.data_dir, config_path=args.config)
    print(f"Motphys Robot Control & Perception Web listening on http://{args.host}:{args.port}")
    uvicorn.run(application, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
