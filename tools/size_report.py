#!/usr/bin/env python3
"""Flash use per component from a GNU ld map file, optionally compared with a base.

Per build:
    python tools/size_report.py MAP TITLE [--json OUT.json] [--base BASE.json]
Summary of several builds (JSON files written with --json, searched recursively):
    python tools/size_report.py --summary CUR_DIR [--base-dir BASE_DIR]

Build with the map enabled, e.g.:
    PLATFORMIO_BUILD_FLAGS="-Wl,-Map,.pio/build/<env>/firmware.map" pio run -e <env>

Counts input sections that end up in flash (code, constants, .data init
values, vectors); RAM-only (.bss, .noinit, COMMON) and debug sections are
skipped. Prints markdown tables, largest first.
"""

import argparse
import collections
import glob
import json
import os
import re

LINE = re.compile(r"^\s*(\S+)?\s+0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)\s+(\S.*)$")
RAM_ONLY = (".bss", ".noinit", "COMMON", ".heap", "._user_heap_stack", ".stack")


def component(path):
    p = path.replace("\\", "/")
    m = re.search(r"/(lib(gcc|m|c|c_nano|nosys|stdc\+\+|stdc\+\+_nano|supc\+\+|supc\+\+_nano))\.a\(", p)
    if m:
        name = m.group(1)
        return {"libgcc": "libgcc (soft-float, int div)", "libm": "libm (math)"}.get(name, name)
    m = re.search(r"/\.pio/build/[^/]+/lib[^/]*/([^/]+)/", p) or re.search(r"/\.pio/libdeps/[^/]+/([^/]+)/", p)
    if m:
        return m.group(1)
    m = re.search(r"/libraries/([^/]+)/", p)  # framework libraries (Wire, SPI, SrcWrapper...)
    if m:
        return m.group(1)
    if "/.pio/build/" in p and "/src/" in p:
        return "sideboard src"
    if "FrameworkArduinoVariant" in p:
        return "core: variant"
    if "FrameworkArduino" in p or "framework-arduino" in p or "libFrameworkArduino" in p:
        return "core"
    if "crt" in p.rsplit("/", 1)[-1]:
        return "startup (crt)"
    return p.rsplit("/", 1)[-1]


def parse(map_path):
    sizes = collections.Counter()
    in_map = False
    pending = None  # section name on its own line, address/size on the next
    with open(map_path, errors="replace") as f:
        for line in f:
            if not in_map:
                in_map = line.startswith("Linker script and memory map")
                continue
            if line.startswith("OUTPUT(") or line.startswith("LOAD linker stubs"):
                break
            m = LINE.match(line)
            if not m:
                s = line.strip()
                pending = s if s.startswith(".") and " " not in s else None
                continue
            name, size, path = (m.group(1) or pending), int(m.group(3), 16), m.group(4).strip()
            pending = None
            if not name or size == 0 or name.startswith("*fill*"):
                continue
            if name.startswith(".debug") or name.startswith(".comment") or name.startswith(".ARM.attributes"):
                continue
            if name.startswith(RAM_ONLY):
                continue
            if not (path.endswith(")") or path.endswith(".o")):
                continue  # symbol lines ("0x... funcname") and linker-script expressions
            sizes[component(path)] += size
    return sizes


def delta(cur, base):
    if base is None:
        return ""
    d = cur - base
    return f"{d:+d}" if d else "0"


def report(args):
    sizes = parse(args.map)
    total = sum(sizes.values())
    base = None
    if args.base and os.path.exists(args.base):
        with open(args.base) as f:
            base = json.load(f)
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"title": args.title, "total": total, "components": dict(sizes)}, f, indent=1)

    has_base = base is not None
    bc = base["components"] if has_base else {}
    print(f"### Flash by component: {args.title}\n")
    if has_base:
        print("| Component | Bytes | vs base | % |\n|---|---:|---:|---:|")
    else:
        print("| Component | Bytes | % |\n|---|---:|---:|")
    names = [n for n, _ in sizes.most_common()] + sorted(set(bc) - set(sizes))  # removed components last
    for name in names:
        size = sizes.get(name, 0)
        pct = f"{100.0 * size / total:.1f}" if total else "0"
        if has_base:
            print(f"| {name} | {size} | {delta(size, bc.get(name, 0))} | {pct} |")
        else:
            print(f"| {name} | {size} | {pct} |")
    if has_base:
        print(f"| **total** | **{total}** | **{delta(total, base['total'])}** | |\n")
    else:
        print(f"| **total** | **{total}** | |\n")


def load_dir(d):
    out = {}
    for p in glob.glob(os.path.join(d, "**", "*.json"), recursive=True):
        with open(p) as f:
            j = json.load(f)
        out[j["title"]] = j["total"]
    return out


def summary(args):
    cur = load_dir(args.summary)
    base = load_dir(args.base_dir) if args.base_dir and os.path.isdir(args.base_dir) else {}
    print("### Flash total per build\n")
    if base:
        print("| Build | Bytes | vs base |\n|---|---:|---:|")
    else:
        print("| Build | Bytes |\n|---|---:|")
    for title in sorted(cur):
        if base:
            print(f"| {title} | {cur[title]} | {delta(cur[title], base.get(title))} |")
        else:
            print(f"| {title} | {cur[title]} |")
    print()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("map", nargs="?", help="linker map file")
    p.add_argument("title", nargs="?", help="title for the table / key in the JSON")
    p.add_argument("--json", help="write the sizes to this JSON file")
    p.add_argument("--base", help="JSON from a previous build to compare with")
    p.add_argument("--summary", metavar="DIR", help="print one table from the JSON files in DIR")
    p.add_argument("--base-dir", help="with --summary: JSON files of the base")
    args = p.parse_args()
    if args.summary:
        summary(args)
    elif args.map:
        args.title = args.title or args.map
        report(args)
    else:
        p.error("MAP or --summary is required")


if __name__ == "__main__":
    main()
