#!/usr/bin/env python3
"""Add a Debian package dependency to the first debian/control Depends stanza."""

from pathlib import Path
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: inject_control_dep.py <control-file> <dependency>", file=sys.stderr)
        return 2

    control_path = Path(sys.argv[1])
    dependency = sys.argv[2]
    lines = control_path.read_text().splitlines()

    for index, line in enumerate(lines):
        if not line.startswith("Depends:"):
            continue

        block_end = index + 1
        while block_end < len(lines) and lines[block_end].startswith(" "):
            block_end += 1

        block = "\n".join(lines[index:block_end])
        if dependency not in block:
            lines[block_end - 1] = lines[block_end - 1].rstrip() + f",\n {dependency}"
        control_path.write_text("\n".join(lines) + "\n")
        return 0

    print(f"{control_path} has no Depends field", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
