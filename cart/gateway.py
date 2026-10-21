"""Gateway (UI, traces, /api proxy) pointed at the cart producer. Replaces the comma SSH setup."""

import argparse
from pathlib import Path

import uvicorn
from drivingbench.gateway.app import create_app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--producer-url", default="http://127.0.0.1:8876")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    # Binds localhost only. From a laptop: ssh -L 8766:127.0.0.1:8766 <cart-pc>
    uvicorn.run(create_app(args.producer_url, repo=args.repo), host="127.0.0.1", port=args.port)


if __name__ == "__main__":
    main()
