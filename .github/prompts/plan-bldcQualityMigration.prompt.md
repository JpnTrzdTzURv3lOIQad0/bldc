## Plan: Resume BLDC Quality Migration

Continue the staged GNU23/quality-gate migration in the actual BLDC checkout from a VS Code WSL window. Preserve the code and editor changes already made; first establish the real Git/rebase state, then finish semantic source cleanup before expecting strict builds or Clang analysis to pass. Use native WSL shell commands from `/mnt/c/external.source/vesc/bldc`; do not wrap commands in `wsl.exe` and do not use Subversion.

**Steps**

**Phase 1: Re-establish session state**
1. In the reopened WSL window, open `/mnt/c/external.source/vesc/bldc`; record `pwd`, `git status --short --branch`, `git rev-parse HEAD`, and whether Git reports a rebase/merge. Prior observations conflict: the original check saw clean `precise-motion` at `2e2be09f5f5ed7bf5eebd7ce8a8f067dc60dd85f`, while a later inspection found `REBASE_HEAD=ef1421ff6adf27b69def6573ecbc9e14f95f875f` and `ORIG_HEAD=f149d407988a25f0b27622f806c060d207b10c25`. Treat status as unknown until verified. Do not reset, abort, continue, or stage a rebase without the user's direction; preserve the current edits.
2. Re-read current `Makefile`, `.vscode/settings.json`, `.vscode/tasks.json`, and `.vscode/extensions.json` before edits because the user reports changes to them. Keep the user's Python environment removal; do not recreate a venv. Settings are locally ignored; retain the current user's changes.
3. Verify `tools/arm-gnu-toolchain-15.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc`, its version, and `command -v bear clang-tidy clangd clang-format python3`. Prior WSL checks found GCC 15.3.1 and system Python; Bear and LLVM tools were not on PATH. Install/pin LLVM 22.1.8 and Bear in WSL if still missing, following project setup guidance.

**Phase 2: Confirm and finish the integration**
4. Inspect existing edits before changing them. The expected integration currently routes `GCC_ANALYZER_TOOLCHAIN` through `ARM_SDK_PREFIX`, selects `-std=gnu23` for C before ChibiOS snapshots options, applies strict response files to owned objects, excludes provisional dependency roots, and runs analyzer-only C object targets with `USE_LTO=no`. A dry-run previously showed 95 analyzer response-file references and zero link recipes. Revalidate this against current files and a small Make dry-run.
5. Confirm the tracked ChibiOS Git rules remove `--no-warn-mismatch`; the production linker path should use fatal warnings and memory reporting. Do not use SVN.
6. Verify `.gitignore` narrowly re-includes `tools/prepare_compdb.py`, `tools/quality_gate.py`, `.vscode/tasks.json`, and `.vscode/extensions.json`, while `.vscode/settings.json` remains ignored. Confirm the copied `compiler/*.rsp`, `compiler/quality.mk`, `quality-policy.json`, `.clang-tidy`, `.clangd`, and `.clang-format` remain present.

**Phase 3: Make owned sources pass without semantic shortcuts**
7. Resume diagnostics from `/tmp/bldc-quality-strict-100_250-retry4.log` if it still exists; otherwise rerun the focused strict `fw_100_250` build in a fresh build directory and capture a concise summary. Latest observed build compiled 59 units, had zero trailing-whitespace errors after cleanup, but failed on about 423 unsuffixed-float diagnostics plus about 89 conversion/API/enumeration diagnostics. Main concentrations were `conf_general.c`, `terminal.c`, `util/utils_math.h`, `main.c`, and several headers.
8. Triage warnings by owning source and contract. Migrate genuine single-precision expressions with explicit `F` literals and float math APIs, and examine integer ranges, discarded volatile/const qualifiers, function prototypes, enum defaults, return values, and null constants. Keep `-fsingle-precision-constant` in the real build until conversion and numerical comparisons are reviewed; never silence the policy or mass-rewrite literals/casts to get a green build. Review RTOS, ISR, MMIO, and packed-wire changes separately.
9. Resolve provisional ownership explicitly, especially locally modified code beneath `libstm32f4`, ChibiOS, `libcanard`, LispBM, and Black Magic. The current exclusion set is provisional, not final policy. Check for basename collisions in ChibiOS's object mapping. Once a board's owned source inventory is complete, set `minimum_translation_units` from its full prepared database; the current value `1` is bootstrap-only.

**Phase 4: Validate the migration**
10. After semantic prerequisites are ready, capture a fresh full build using Bear for each required board/configuration; do not reuse incremental or partial databases. Prepare with the exact GCC/G++ 15.3.1 executables. The prepare step is expected to reject `-fsingle-precision-constant` until it is removed from the real build after source conversion.
11. Run optimized production warning builds and the separate analyzer-only C object job (`GCC_ANALYZER=yes`, `USE_LTO=no`) independently. Inspect real compiler commands and banners. Run `quality_gate.py` with LLVM 22.1.8 for all selected commands after reviewing ownership/counts. Run package regression tests with WSL system Python; previously all 10 `unittest` tests passed.
12. Extend to required board, feature, debug/assert, and release configurations. Review image/map sizes, region margins, stack reports, tests, startup/telemetry/watchdog behavior, control boundaries, ISR timing/jitter, and hardware behavior before release use. No hardware/runtime acceptance has been performed in this session.

**Relevant files**
- `/mnt/c/external.source/vesc/bldc/Makefile` — configured analyzer compiler and lint target; recent user edits must be reread.
- `/mnt/c/external.source/vesc/bldc/make/fw.mk` — C-only dialect, quality response files, source ownership and analyzer object target.
- `/mnt/c/external.source/vesc/bldc/ChibiOS_21.11.5/os/common/startup/ARMCMx/compilers/GCC/mk/rules.mk` — mismatch-warning suppression removal.
- `/mnt/c/external.source/vesc/bldc/.gitignore`, `/mnt/c/external.source/vesc/bldc/.vscode/` — trackable shared quality tooling; preserve user-local settings.
- `/mnt/c/external.source/vesc/bldc/compiler/`, `/mnt/c/external.source/vesc/bldc/tools/prepare_compdb.py`, `/mnt/c/external.source/vesc/bldc/tools/quality_gate.py`, `/mnt/c/external.source/vesc/bldc/quality-policy.json`, `/mnt/c/external.source/vesc/bldc/.clang-tidy`, `/mnt/c/external.source/vesc/bldc/.clangd`, `/mnt/c/external.source/vesc/bldc/.clang-format` — staged quality package and policy.
- `/mnt/c/external.source/vesc/bldc/conf_general.c`, `/mnt/c/external.source/vesc/bldc/terminal.c`, `/mnt/c/external.source/vesc/bldc/util/utils_math.h`, `/mnt/c/external.source/vesc/bldc/main.c` — largest observed strict-warning concentrations; expand to other paths from the latest log.

**Verification**
1. Reconcile the Git/rebase state and retain user changes; no destructive Git operation without explicit approval.
2. Make dry-run confirms GNU23 on C only, strict response files on owned C/C++ objects, analyzer response flags on C objects, no analyzer link, and no `--no-warn-mismatch`.
3. Run the 10 package unit tests with system Python, the strict optimized board build, the separate analyzer object job, and the full tidy gate once prerequisites and semantic findings are resolved.
4. Inspect fresh compile-command coverage, policy counts, compiler/LLVM versions, link maps/resource margins, and hardware evidence for each required release configuration.

**Decisions**
- All firmware build, database capture, analysis, and clangd work runs inside WSL.
- Keep the current single-precision compiler flag until code is explicitly migrated and behavior compared.
- Treat third-party ownership exclusions and the compile-unit floor as provisional.
- No release certification, full Clang gate, full board matrix, or hardware validation is complete yet.
