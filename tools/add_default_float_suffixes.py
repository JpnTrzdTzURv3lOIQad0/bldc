#!/usr/bin/env python3
"""Add F suffixes to numeric MCCONF overrides that are float defaults."""

import argparse
from pathlib import Path
import re


NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
DEFAULT_FLOAT_RE = re.compile(
    rf"^[ \t]*#[ \t]*define[ \t]+(?P<name>MCCONF_[A-Za-z0-9_]+)"
    rf"[ \t]+{NUMBER}F(?=[ \t]*(?://|/\*|$))",
    re.MULTILINE,
)
DEFINE_LITERAL_RE = re.compile(
    rf"(?P<prefix>[ \t]*#[ \t]*define[ \t]+(?P<name>[A-Za-z_][A-Za-z0-9_]*)"
    rf"[ \t]+)(?P<literal>{NUMBER})(?P<float_suffix>[fF]?)"
    rf"(?P<tail>[ \t]*(?://.*|/\*.*\*/[ \t]*)?)"
)


def default_float_macros(default_header):
    text = default_header.read_text(encoding="utf-8")
    return {match.group("name") for match in DEFAULT_FLOAT_RE.finditer(text)}


def update_header(header, float_macros, dry_run):
    with header.open("r", encoding="utf-8", newline="") as source:
        lines = source.readlines()

    updated_lines = []
    replacements = 0
    for line in lines:
        ending = "\r\n" if line.endswith("\r\n") else "\n" if line.endswith("\n") else ""
        body = line[:-len(ending)] if ending else line
        match = DEFINE_LITERAL_RE.fullmatch(body)
        if match and match.group("name") in float_macros:
            literal = match.group("literal")
            suffix = match.group("float_suffix")
            if re.fullmatch(r"[+-]?\d+", literal):
                replacement = literal + ".0F"
            elif not suffix:
                replacement = literal + "F"
            else:
                replacement = None

            if replacement is not None:
                body = (
                    match.group("prefix")
                    + replacement
                    + match.group("tail")
                ).rstrip(" \t")
                replacements += 1
        updated_lines.append(body + ending)

    if replacements and not dry_run:
        with header.open("w", encoding="utf-8", newline="") as destination:
            destination.writelines(updated_lines)
    return replacements


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Report matching definitions without modifying files",
    )
    options = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    default_header = root / "motor" / "mcconf_default.h"
    hardware_headers = sorted((root / "hwconf").rglob("*.h"))
    float_macros = default_float_macros(default_header)

    changed_files = 0
    replacements = 0
    for header in hardware_headers:
        count = update_header(header, float_macros, options.dry_run)
        if count:
            changed_files += 1
            replacements += count
            print(f"{header.relative_to(root)}: {count}")

    action = "Would add" if options.dry_run else "Added"
    print(f"{action} F to {replacements} definitions in {changed_files} headers.")


if __name__ == "__main__":
    main()
