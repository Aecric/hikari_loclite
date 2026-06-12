#!/usr/bin/env python3
"""Resolve the Livox ROS driver deb asset from a GitHub release JSON."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--release-json", required=True)
    parser.add_argument("--ros-distro", required=True)
    parser.add_argument("--ubuntu-codename", required=True)
    parser.add_argument("--arch", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    release = json.loads(Path(args.release_json).read_text(encoding="utf-8"))
    assets = release.get("assets") or []

    strict = re.compile(
        r"^ros-%s-livox-ros-driver2_.*-0%s_%s\.deb$"
        % (
            re.escape(args.ros_distro),
            re.escape(args.ubuntu_codename),
            re.escape(args.arch),
        )
    )
    fallback = re.compile(
        r"^ros-%s-livox-ros-driver2_.*_%s\.deb$"
        % (re.escape(args.ros_distro), re.escape(args.arch))
    )

    matches = [asset for asset in assets if strict.match(asset.get("name", ""))]
    matches = matches or [asset for asset in assets if fallback.match(asset.get("name", ""))]
    if not matches:
        names = ", ".join(asset.get("name", "") for asset in assets) or "<none>"
        print(
            "no livox deb asset for "
            f"ros={args.ros_distro}, codename={args.ubuntu_codename}, "
            f"arch={args.arch}; assets: {names}",
            file=sys.stderr,
        )
        return 1

    asset = matches[0]
    name = asset["name"]
    url = asset["browser_download_url"]
    Path(args.output).write_text(f"{name}\t{url}\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
