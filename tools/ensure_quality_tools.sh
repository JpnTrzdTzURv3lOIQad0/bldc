#!/usr/bin/env bash
set -euo pipefail

case "${1:-}" in
    --check) check_only=true ;;
    '') check_only=false ;;
    --help)
        printf 'Usage: bash tools/ensure_quality_tools.sh [--check]\n'
        printf 'Install missing Debian/Ubuntu build dependencies and ARM GCC; verify pinned LLVM.\n'
        printf 'Override tool locations with ARM_SDK_DIR and LLVM_BIN. --check never installs.\n'
        exit 0
        ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
esac
if (( $# > 1 )); then
    printf 'Expected at most one argument.\n' >&2
    exit 2
fi

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
default_sdk="$repo_root/tools/arm-gnu-toolchain-15.3.rel1-x86_64-arm-none-eabi"
arm_sdk=${ARM_SDK_DIR:-$default_sdk}
llvm_bin=${LLVM_BIN:-$HOME/.local/opt/llvm-22.1.8/bin}
packages=()
declare -A selected_packages=()
for requirement in make:build-essential gcc:build-essential g++:build-essential \
    bear:bear python3:python3 git:git wget:wget tar:tar xz:xz-utils unzip:unzip cmake:cmake; do
    executable=${requirement%%:*}
    package=${requirement#*:}
    if ! command -v "$executable" >/dev/null 2>&1; then
        printf 'Missing host tool: %s\n' "$executable" >&2
        if [[ -z ${selected_packages[$package]:-} ]]; then
            packages+=("$package")
            selected_packages[$package]=yes
        fi
    fi
done

if (( ${#packages[@]} > 0 )); then
    if "$check_only"; then
        printf 'Run this script without --check to install the missing packages.\n' >&2
        exit 1
    fi
    if ! command -v apt-get >/dev/null 2>&1; then
        printf 'Automatic host installation requires Debian/Ubuntu apt-get.\n' >&2
        exit 1
    fi
    privilege=()
    if (( EUID != 0 )); then
        if ! command -v sudo >/dev/null 2>&1 || ! sudo -n true 2>/dev/null; then
            printf 'Run in your terminal: sudo apt-get update && sudo apt-get install -y' >&2
            printf ' %q' "${packages[@]}" >&2
            printf '\nThen rerun this script. No password will be requested by this script.\n' >&2
            exit 1
        fi
        privilege=(sudo -n)
    fi
    "${privilege[@]}" apt-get update
    "${privilege[@]}" apt-get install -y "${packages[@]}"
fi

if [[ ! -x "$arm_sdk/bin/arm-none-eabi-gcc" || ! -x "$arm_sdk/bin/arm-none-eabi-g++" ]]; then
    if "$check_only" || [[ "$arm_sdk" != "$default_sdk" ]]; then
        printf 'Missing ARM toolchain at %s; install it or set ARM_SDK_DIR.\n' "$arm_sdk" >&2
        exit 1
    fi
    make -C "$repo_root" arm_sdk_install
fi

gcc_version=$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["gcc_version"])' "$repo_root/quality-policy.json")
llvm_version=$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["llvm_version"])' "$repo_root/quality-policy.json")
for compiler in arm-none-eabi-gcc arm-none-eabi-g++; do
    if [[ $("$arm_sdk/bin/$compiler" -dumpfullversion) != "$gcc_version" || \
          $("$arm_sdk/bin/$compiler" -dumpmachine) != arm-none-eabi ]]; then
        printf '%s must be ARM GCC %s; refusing to replace an existing toolchain.\n' "$compiler" "$gcc_version" >&2
        exit 1
    fi
done
for tool in clangd clang-tidy clang-format; do
    if [[ ! -x "$llvm_bin/$tool" ]]; then
        printf 'Install LLVM %s including %s at %s, or set LLVM_BIN to its bin directory.\n' \
            "$llvm_version" "$tool" "$llvm_bin" >&2
        exit 1
    fi
    banner=$("$llvm_bin/$tool" --version)
    if ! grep -Eq "version[[:space:]]+${llvm_version//./\\.}([[:space:]]|$)" <<< "$banner"; then
        printf '%s must be LLVM %s; found: %s\n' "$tool" "$llvm_version" "$banner" >&2
        exit 1
    fi
done
printf 'Required host tools and pinned toolchains are available.\n'
printf 'ARM_SDK_DIR=%q\nLLVM_BIN=%q\n' "$arm_sdk" "$llvm_bin"
