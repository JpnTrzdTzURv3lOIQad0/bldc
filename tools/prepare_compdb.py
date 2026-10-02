#!/usr/bin/env python3
"""Prepare a separate Clang database from real GNU ARM build commands.

No external Python packages. Does not modify the input database or firmware.
Run in the build environment: do not mix Windows tools with WSL paths.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys

CXX_SUFFIXES = {".cc", ".cpp", ".cxx", ".C"}
SOURCE_SUFFIXES = CXX_SUFFIXES | {".c"}
# Audited diagnostics with no identical Clang spelling/implementation.
GCC_ONLY_WARNINGS = {
    "-Wformat-overflow=2", "-Wformat-truncation=2", "-Wformat-signedness",
    "-Warith-conversion", "-Wcast-align=strict", "-Wimplicit-fallthrough=5", "-Wshadow=global",
    "-Wtrampolines", "-Warray-bounds=2", "-Wstringop-overflow=2",
    "-Wstringop-truncation", "-Wduplicated-cond", "-Wduplicated-branches",
    "-Wlogical-op", "-Wpacked-not-aligned", "-Wstrict-aliasing=3",
    "-Wshift-overflow=2", "-Wjump-misses-init", "-Wunsuffixed-float-constants",
    "-Wanalyzer-too-complex", "-Wanalyzer-symbol-too-complex",
    "-Wtrailing-whitespace=any",
}
# These affect compilation products/optimization, not the source language model.
DROP_EXACT = GCC_ONLY_WARNINGS | {
    "-c", "-S", "-MD", "-MMD", "-MP", "-MG", "-Werror",
    "-fanalyzer", "-fstack-usage", "-ffat-lto-objects", "-fno-fat-lto-objects",
    "-fuse-linker-plugin", "-flto", "-fno-lto", "-mthumb-interwork",
    "-fno-tree-loop-distribute-patterns", "-fno-reorder-functions",
}
DROP_PREFIXES = (
    "-flto=", "-freorder-blocks-algorithm=", "-falign-functions=",
    "-falign-jumps=", "-falign-labels=", "-falign-loops=",
    "-Wstack-usage=", "-Wframe-larger-than=", "-Wl,", "-Wa,",
)
PROBE_FLAGS = (
    "-m", "-std=", "--sysroot=", "-isysroot", "-fshort-enums",
    "-fshort-wchar", "-fsigned-char", "-funsigned-char", "-ffreestanding",
)


def source_path(entry):
    return (Path(entry["directory"]) / entry["file"]).resolve()


def require_same_driver(recorded, compiler, cwd):
    """Do not silently analyze a GCC 14 database using GCC 15 headers."""
    if Path(recorded).is_absolute() or "/" in recorded or "\\" in recorded:
        resolved = (cwd / recorded).resolve()
    else:
        found = shutil.which(recorded)
        if found is None:
            raise ValueError(f"Recorded compiler is not on PATH: {recorded}; recapture using absolute paths")
        resolved = Path(found).resolve()
    if resolved != compiler.resolve():
        raise ValueError(f"Recorded compiler differs from selected compiler: {resolved} != {compiler}; recapture the build database")


def expand_response(args, cwd, depth=0, response_files=None):
    if depth > 8:
        raise ValueError("Response-file nesting exceeds 8")
    result = []
    for arg in args:
        if arg.startswith("@"):
            path = (cwd / arg[1:]).resolve()
            content = path.read_bytes()
            if response_files is not None:
                response_files[str(path)] = hashlib.sha256(content).hexdigest()
            text = content.decode("utf-8-sig")
            result.extend(expand_response(shlex.split(text, posix=True), cwd, depth + 1, response_files))
        else:
            result.append(arg)
    return result


def normalize(args):
    kept, removed = [], []
    index = 0
    while index < len(args):
        arg = args[index]
        if arg == "-fsingle-precision-constant":
            raise ValueError(
                "Clang cannot model -fsingle-precision-constant faithfully. "
                "Migrate intended float literals to F suffixes and appropriate "
                "math calls, remove the option from the REAL build, then "
                "revalidate numerical behavior. Do not silently drop this flag."
            )
        if arg.startswith(("-specs=", "--specs=")) or arg in {"-specs", "--specs"}:
            # Specs files can inject preprocessing/ABI options as well as link flags.
            # Only the known link-only newlib no-syscalls specs is allowed here.
            if arg in {"-specs=nosys.specs", "--specs=nosys.specs"}:
                removed.append(arg)
                index += 1
                continue
            raise ValueError(f"Review and expand unsupported specs file: {arg}")
        if arg in {"-o", "-MF", "-MT", "-MQ"}:
            if index + 1 == len(args):
                raise ValueError(f"Missing value after {arg}")
            removed.extend(args[index:index + 2])
            index += 2
            continue
        if arg in DROP_EXACT or arg.startswith(DROP_PREFIXES):
            removed.append(arg)
        elif any(arg.startswith(prefix) and arg != prefix for prefix in ("-MF", "-MT", "-MQ")):
            removed.append(arg)
        else:
            kept.append(arg)
        index += 1
    return kept, removed


def query_includes(compiler, args, language, cwd):
    # Explicit, user-specified executable; never execute a compiler from the DB.
    flags = []
    index = 0
    while index < len(args):
        arg = args[index]
        if arg in {"--sysroot", "-isysroot"}:
            flags.extend(args[index:index + 2])
            index += 2
            continue
        if arg.startswith(PROBE_FLAGS):
            flags.append(arg)
        index += 1
    process = subprocess.run(
        [str(compiler), *flags, "-E", "-v", "-x", language, "-"],
        input="", text=True, capture_output=True, cwd=cwd, check=True,
    )
    paths, active = [], False
    for line in process.stderr.splitlines():
        if "#include <...> search starts here:" in line:
            active = True
        elif "End of search list." in line:
            active = False
        elif active:
            path = (cwd / line.strip()).resolve()
            if not path.is_dir():
                raise ValueError(f"GCC include directory does not exist: {path}")
            paths.append(str(path))
    if not paths:
        raise ValueError("GCC include discovery returned no paths")
    return paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="Output directory")
    parser.add_argument("--gcc", type=Path, required=True)
    parser.add_argument("--gxx", type=Path)
    parser.add_argument("--gcc-version", default="15.3.1", help="Exact validated GCC version (also set gcc_version in policy)")
    parser.add_argument("--posix-command", action="store_true", help="Allow POSIX-shell command entries")
    options = parser.parse_args()
    options.input = options.input.resolve()
    options.output = options.output.resolve()
    if options.input == options.output / "compile_commands.json":
        raise ValueError("Input and output databases must differ")
    compiler_paths = [options.gcc] + ([options.gxx] if options.gxx else [])
    compiler_records = []
    for compiler in compiler_paths:
        if not compiler.is_absolute() or not compiler.is_file():
            raise ValueError(f"Supply an absolute, existing trusted compiler: {compiler}")
        machine = subprocess.check_output([str(compiler), "-dumpmachine"], text=True).strip()
        version = subprocess.check_output([str(compiler), "-dumpfullversion"], text=True).strip()
        if machine != "arm-none-eabi" or version != options.gcc_version:
            raise ValueError(f"Expected Arm GCC {options.gcc_version}, got {machine} {version}")
        compiler_records.append({"path": str(compiler.resolve()), "target": machine,
                                 "version": version,
                                 "sha256": hashlib.sha256(compiler.read_bytes()).hexdigest(),
                                 "banner": subprocess.check_output([str(compiler), "--version"], text=True).strip()})
    original = options.input.read_bytes()
    entries = json.loads(original)
    output, report = [], []
    response_files = {}
    cache = {}
    for entry in entries:
        source = source_path(entry)
        if source.suffix not in SOURCE_SUFFIXES:
            continue
        cwd = Path(entry["directory"]).resolve()
        if not source.is_file() or not cwd.is_dir():
            raise ValueError(f"Stale database path: {source}")
        args = entry.get("arguments")
        if args is None:
            if not options.posix_command:
                raise ValueError("Use an arguments-array database, or --posix-command for POSIX shell syntax")
            args = shlex.split(entry["command"], posix=True)
        if not args or not re.search(r"arm-none-eabi-(gcc|g\+\+)(\.exe)?$", args[0]):
            raise ValueError(f"Expected direct ARM compiler (expand launchers first): {args[:2]}")
        args = expand_response(args[1:], cwd, response_files=response_files)
        normalized, removed = normalize(args)
        is_cpp = source.suffix in CXX_SUFFIXES
        compiler = options.gxx if is_cpp else options.gcc
        if compiler is None:
            raise ValueError(f"C++ source requires --gxx: {source}")
        # args[0] was removed above; check the original recorded executable.
        recorded_args = entry.get("arguments") or shlex.split(entry["command"], posix=True)
        require_same_driver(recorded_args[0], compiler, cwd)
        language = "c++" if is_cpp else "c"
        key = (str(compiler), str(cwd), tuple(normalized), language)
        if key not in cache:
            cache[key] = query_includes(compiler, normalized, language, cwd)
        includes = cache[key]
        extra = ["--target=arm-none-eabi"]
        # Disable host/GCC autodiscovery; use only the explicit queried target paths.
        extra += ["-nostdinc"]
        if is_cpp:
            extra += ["-nostdinc++"]
        for include in includes:
            extra += ["-isystem", include]
        output.append({"directory": str(cwd), "file": str(source),
                       "arguments": [str(compiler), *normalized, *extra]})
        report.append({"file": str(source), "removed": removed, "system_includes": includes})
    if not output:
        raise ValueError("Database has no C/C++ translation units")
    options.output.mkdir(parents=True, exist_ok=True)
    output_bytes = (json.dumps(output, indent=2) + "\n").encode()
    (options.output / "compile_commands.json").write_bytes(output_bytes)
    manifest = {"source_database": str(options.input),
                "source_sha256": hashlib.sha256(original).hexdigest(),
                "prepared_sha256": hashlib.sha256(output_bytes).hexdigest(),
                "response_files": response_files,
                "compilers": compiler_records,
                "translation_units": report}
    (options.output / "preparation-report.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared {len(output)} translation units; review {options.output / 'preparation-report.json'}")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        print(f"Preparation failed: {error}", file=sys.stderr)
        sys.exit(2)
