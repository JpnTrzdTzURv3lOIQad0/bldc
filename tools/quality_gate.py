#!/usr/bin/env python3
"""Run pinned clang-tidy on every selected real compilation command; fail closed."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile

from prepare_compdb import CXX_SUFFIXES, SOURCE_SUFFIXES, source_path


def below(path, roots):
    return any(path == root or root in path.parents for root in roots)


def validate_command(entry, policy):
    args = entry.get("arguments")
    if not args:
        raise ValueError("Prepared database needs arguments arrays")
    if any(arg.startswith("@") for arg in args):
        raise ValueError("Unexpanded response file in prepared database")
    for arg in args:
        if arg == "-w" or arg.startswith("-Wno-"):
            if arg not in policy.get("approved_compile_suppressions", []):
                raise ValueError(f"Unreviewed diagnostic suppression: {arg}")
    def last(prefix):
        values = [arg[len(prefix):] for arg in args if arg.startswith(prefix)]
        return values[-1] if values else None
    source = source_path(entry)
    dialect = policy["cxx_standard" if source.suffix in CXX_SUFFIXES else "c_standard"]
    expected = {"-std=": dialect, "--target=": policy["target"],
                "-mcpu=": policy["cpu"], "-mfpu=": policy["fpu"],
                "-mfloat-abi=": policy["float_abi"]}
    for prefix, value in expected.items():
        if last(prefix) != value:
            raise ValueError(f"{source}: expected {prefix}{value}, found {last(prefix)}")
    if "-mthumb" not in args or "-marm" in args:
        raise ValueError(f"{source}: Cortex-M requires Thumb")
    if "-fsingle-precision-constant" in args:
        raise ValueError("Unsupported floating literal semantics in prepared DB")
    if not source.is_file():
        raise ValueError(f"Source does not exist: {source}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--clang-tidy", default="clang-tidy")
    parser.add_argument("--jobs", type=int, default=1)
    options = parser.parse_args()
    if options.jobs < 1:
        raise ValueError("jobs must be positive")
    policy_path = options.policy.resolve()
    root = policy_path.parent
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    profile = (root / policy["profile"]).resolve()
    if not profile.is_file():
        raise ValueError(f"Missing profile: {profile}")
    version = subprocess.check_output([options.clang_tidy, "--version"], text=True)
    if re.search(r"\bversion\s+" + re.escape(policy["llvm_version"]) + r"\b", version) is None:
        raise ValueError(f"Expected LLVM {policy['llvm_version']}; got {version.strip()}")
    subprocess.run([options.clang_tidy, f"--config-file={profile}", "--verify-config"], check=True)
    database = options.database.resolve() / "compile_commands.json"
    raw = database.read_bytes()
    manifest_path = database.parent / "preparation-report.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    compiler_records = manifest.get("compilers", [])
    if not compiler_records:
        raise ValueError("Missing compiler identity; regenerate analysis database")
    for compiler in compiler_records:
        if compiler["version"] != policy["gcc_version"] or compiler["target"] != policy["target"]:
            raise ValueError("Compiler identity does not match policy; regenerate using the pinned toolchain")
        if hashlib.sha256(Path(compiler["path"]).read_bytes()).hexdigest() != compiler["sha256"]:
            raise ValueError("Compiler executable changed; regenerate analysis database")
    if hashlib.sha256(raw).hexdigest() != manifest["prepared_sha256"]:
        raise ValueError("Prepared database changed; regenerate it")
    source_db = Path(manifest["source_database"])
    if hashlib.sha256(source_db.read_bytes()).hexdigest() != manifest["source_sha256"]:
        raise ValueError("Build database changed; regenerate analysis database")
    for filename, expected_hash in manifest.get("response_files", {}).items():
        if hashlib.sha256(Path(filename).read_bytes()).hexdigest() != expected_hash:
            raise ValueError(f"Response file changed; regenerate analysis database: {filename}")
    entries = json.loads(raw)
    owned = [(root / path).resolve() for path in policy["owned_roots"]]
    excluded = [(root / path).resolve() for path in policy["exclude_roots"]]
    if not owned:
        raise ValueError("owned_roots must not be empty")
    selected, omitted = [], []
    for entry in entries:
        source = source_path(entry)
        if source.suffix not in SOURCE_SUFFIXES:
            continue
        if below(source, owned) and not below(source, excluded):
            validate_command(entry, policy)
            selected.append(entry)
        else:
            omitted.append(str(source))
    count = len(selected)
    minimum = policy["minimum_translation_units"]
    if minimum < 1 or count < minimum:
        raise ValueError(f"Only {count} commands selected; expected at least {max(minimum, 1)}")
    print(f"Selected {count} commands; excluded {len(omitted)} (recorded in gate-report.json)", flush=True)
    # One DB per command: multiple configurations for one source are ALL checked.
    def run_one(entry):
        with tempfile.TemporaryDirectory(prefix="embedded-tidy-") as directory:
            Path(directory, "compile_commands.json").write_text(json.dumps([entry]), encoding="utf-8")
            process = subprocess.run(
                [options.clang_tidy, f"-p={directory}", f"--config-file={profile}",
                 "--warnings-as-errors=*", str(source_path(entry))],
                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            )
            return str(source_path(entry)), process.returncode, process.stdout
    results = []
    with ThreadPoolExecutor(max_workers=options.jobs) as pool:
        for source, returncode, output in pool.map(run_one, selected):
            print(output, end="", flush=True)
            results.append({"file": source, "exit_code": returncode, "output": output})
    report = {"llvm": version.strip(), "compilers": compiler_records,
              "profile_sha256": hashlib.sha256(profile.read_bytes()).hexdigest(),
              "selected_commands": count, "excluded": omitted, "results": results}
    (database.parent / "gate-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    failed = sum(result["exit_code"] != 0 for result in results)
    print(f"Analysis gate: {count - failed}/{count} commands passed")
    return 1 if failed else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, KeyError, OSError, subprocess.SubprocessError) as error:
        print(f"Analysis gate failed: {error}", file=sys.stderr)
        sys.exit(2)
