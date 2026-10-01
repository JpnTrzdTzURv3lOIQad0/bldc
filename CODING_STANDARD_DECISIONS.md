# Embedded C Coding Standard Decisions

## Scope

This draft applies C-language guidance to BLDC. It uses BARR-C:2018, ESCR C 3.0, SEI CERT C 2016, JPL C 1.0, and GNU Coding Standards (last updated 2026-04-27) as references. The current GNU manual is available in the workspace as `gnustandards-2026.md`; `gnustandards.md` is the historical 2000 edition. AUTOSAR C++14 and ESCR C++ guidance are not C standards and are not enabled as BLDC C checks. The root configuration also reaches C++ unit-test translation units; their compiler-selected language mode remains authoritative.

Mark one choice per item. The checked choice is the current recommendation and describes the current `.clang-tidy` behavior where applicable. These are project policy selections, not claims of complete standards compliance.

## Guidance Priority

The attached 2000 GNU document is retained for history; the current GNU Coding Standards source identifies its latest update as April 27, 2026. GNU describes its rules as the minimum for GNU packages and says projects should set additional package standards, so this document is reference guidance rather than BLDC's governing embedded policy.

For BLDC, apply this order when guidance conflicts: (1) target hardware and real-time requirements; (2) safe, predictable embedded guidance from BARR-C, ESCR C, CERT C, and JPL; (3) GNU general C guidance where compatible. Do not carry GNU's general-purpose advice to dynamically allocate arbitrary-sized data into real-time firmware paths. GNU 2026 also warns against contorting code merely to silence noisy diagnostics. Record project choices below rather than treating any source as universally controlling.

## Decisions

### C dialect and extensions

- [x] Keep the firmware's existing `-std=gnu99`; confine and document compiler/GNU extensions at hardware boundaries.
- [ ] Require strict `-std=c99` and remove GNU language extensions.

BLDC's `make/fw.mk` selects `gnu99` and defines `_GNU_SOURCE`. BARR-C 1.1 requires C99 and says extensions should be localized. GNU Coding Standards 3.3-3.4 accept C99 and extensions when useful, but recommend conditional use when other compilers must be supported; GNU 2026 specifically includes VLAs among extensions to consider conditionally. Keep `gnu99` for the existing GCC firmware toolchain, localize extensions where practical, and do not change dialect without checking target support and hardware code.

### String and buffer APIs

- [x] Require explicit capacity and input validation through project wrappers or equivalent checked code; do not require Annex K names universally.
- [ ] Require C11 Annex K functions such as `strcpy_s` after confirming every supported libc provides them.
- [ ] Permit existing string APIs without a uniform capacity-validation rule.

ESCR C 3.0's secure-coding guidance favors bounds-checking interfaces; CERT C discusses Annex K as optional and incompletely implemented. GNU Coding Standards 2026 section 5.7 favors standard interfaces where possible and Gnulib when portability requires it. A size-carrying project wrapper is the proposed middle ground; do not require Annex K without confirming target-libc support. `cert-*` and `clang-analyzer-*` provide useful findings, but do not prove every string operation is bounded.

### Variable-length arrays

- [x] Reject VLAs in firmware translation units using `-Wvla`; review and selectively gate the diagnostic after establishing the firmware baseline.
- [ ] Allow VLAs only with validated positive bounds and a documented worst-case stack calculation.

BARR-C's C99 baseline permits the language feature but does not require its use. GNU Coding Standards 2026 section 3.4 says GNU extensions such as VLAs should be conditional when portability requires other compilers. ESCR C 3.0 R3.1.4 and CERT C ARR32-C emphasize invalid bounds and stack-exhaustion risk. The checked choice favors predictable stack usage.

### Pointer arithmetic

- [x] Permit pointer arithmetic only within the same array/object with bounds established by the caller; prefer indexing where it improves clarity.
- [ ] Prohibit pointer increments/offsets and require array indexing for all accesses.

ESCR C 3.0 R1.3.1 explicitly presents both alternatives. BLDC performs low-level buffer and hardware work, so the proposal keeps bounded C pointer arithmetic. `-Wpointer-arith` rejects non-standard `void *` arithmetic; it does not prove array bounds. Bounds and pointer provenance still need review/static analysis.

### Signed bitwise operands

- [x] Require unsigned types for bit sequences, masks, and shifts; document any hardware representation conversion.
- [ ] Permit signed bitwise operations where their behavior is reviewed and documented.

BARR-C 5.3 and ESCR C 3.0 R2.6.2 favor unsigned operands for bit manipulation. The HICPP `hicpp-signed-bitwise` check is C++-oriented and was removed. Stock C clang-tidy plus `-Wsign-conversion` does not reliably enforce this rule; retain it as a review requirement or add a dedicated C analyzer if automatic enforcement is required.

### Dynamic allocation

- [x] Do not allocate dynamically in interrupt or real-time control paths; allow only documented initialization-time allocation with bounded capacity and failure handling.
- [ ] Prohibit heap allocation everywhere.
- [ ] Permit run-time allocation under a documented allocator and latency budget.

JPL C 1.0 Rule 5 prohibits allocation after task initialization. GNU Coding Standards 2026 sections 2.1 and 4.2 recommend dynamic allocation to avoid arbitrary limits in general-purpose programs. That advice prioritizes unbounded input sizes, not hard real-time execution or bounded MCU memory. For firmware, timing, fragmentation, and failure behavior favor the checked restriction. Clang-tidy's allocation checks detect defects; they cannot enforce this lifecycle policy by themselves.

### Recursion and `goto`

- [x] Prohibit direct/indirect recursion; allow `goto` only for a reviewed cleanup/error path when it is clearer and stays within the function.
- [ ] Prohibit both recursion and `goto` without exceptions.
- [ ] Permit bounded recursion and BARR-C's restricted forward `goto` use.

JPL C 1.0 Rules 4 and 11 prohibit recursion and `goto`; BARR-C 1.7 discourages `goto` but permits restricted exceptional use. The existing broad `misc-*` family includes `misc-no-recursion`, with analysis limitations for indirect calls through function pointers. Clang-tidy has no equivalent C `goto` prohibition check.

### Terminating loop bounds

- [x] Require a statically verifiable upper bound for every loop intended to terminate, and review the bound against the input/data capacity.
- [ ] Accept runtime-dependent termination conditions when input validation and a worst-case execution bound are documented.

JPL C 1.0 Rule 3 favors verifiable loop bounds. BARR-C 8.4 prohibits magic loop endpoints, while ESCR C emphasizes bounds and checked data sizes. `bugprone-infinite-loop` catches only some defects; clang-tidy cannot prove a worst-case bound for every loop. This is a review and target-test requirement.

### Numeric literals and register values

- [x] Use named constants for limits, sizes, timeouts, and protocol values; allow powers-of-two masks and bit-field widths.
- [ ] Apply `readability-magic-numbers` uniformly, including register masks and bit positions.
- [ ] Limit the rule to loop bounds, as BARR-C 8.4 does.

ESCR C 3.0 takes a broader symbolic-constant approach than BARR-C 8.4's specific loop-bound rule. Uniform magic-number diagnostics can create noise in register-level code, while unexplained operational thresholds are risky. The profile's `readability-magic-numbers.IgnorePowersOf2IntegerValues` option implements the checked compromise; other masks should use named constants.

### Naming prefixes

- [x] Keep the current lower-case identifiers, upper-case macros/enumerators, and `_t` typedef suffix.
- [ ] Also enforce BARR-C prefixes for globals (`g_`), pointers (`p_`/`pp_`), and integer Boolean variables (`b_`) after auditing existing and public APIs.

The profile enforces the common lexical style. Prefix rules improve role visibility, but applying them mechanically to established firmware symbols and hardware APIs may produce churn; review before enabling them.

### Warning policy

- [x] Keep broad diagnostics enabled as warnings; review and baseline them before promoting selected checks or changed-code findings to errors.
- [ ] Treat every diagnostic from every selected family as an error immediately.

BARR-C calls for automated scans and recommends changing legacy code module by module. ESCR C recommends static analysis and appropriate compiler warnings. GNU Coding Standards 2026 section 5.3 explicitly cautions against making code ugly merely to satisfy static-analysis warnings, naming `-Wconversion` and `-Wundef` as examples that can generate false alarms. Keep these useful diagnostics visible, but do not add unnecessary casts, wrappers, or logic solely to silence them. The config therefore clears blanket `WarningsAsErrors`; review the first full compile-database baseline before choosing narrow CI error gates.

### Stack and execution budgets

- [x] Set stack, heap, interrupt-stack, and control-loop execution budgets from the actual MCU/linker map and scheduling requirements; verify them with target builds, stack analysis, and timing tests.
- [ ] Adopt a fixed generic byte/cycle threshold before the target-specific budgets are documented.

JPL's predictable-execution rules and the embedded reliability/efficiency practices in ESCR C favor verifiable resource bounds. No universal numeric threshold is safe to invent here: it depends on the MCU, RTOS task configuration, interrupt nesting, compiler options, and control-loop deadline. The current clang-tidy profile does not enforce these budgets.

## Coverage Notes

The C-focused `.clang-tidy` retains the original broad `bugprone-*`, `clang-analyzer-*`, `cert-*`, `performance-*`, `portability-*`, `misc-*`, and `readability-*` families, plus compiler diagnostics. It adds C-relevant prototype-style, pointer-extension, alignment, string-const, and floating-promotion warnings. `make/fw.mk` adds `-Wmissing-prototypes` to firmware-only `CWARN`, keeping the C-specific warning off C++ unit-test builds. C++-oriented check groups/options were removed. Diagnostics remain warnings by default, consistent with GNU 2026's caution about noisy warning flags; embedded safety findings still require review and selective CI gating.

Clang-tidy does not enforce the BARR-C formatting rules (80-column width, Allman brace placement, tabs, or line endings), comment quality, module/file structure, cast rationale, signed-bitwise policy for C, ISR/vector correctness, register layout, interrupt race freedom, worst-case execution time, or stack bounds. Keep formatter/text checks, compiler and linker validation, static analysis, tests, and reviews for those requirements. Do not treat this profile as MISRA, CERT, JPL, or BARR-C certification.
