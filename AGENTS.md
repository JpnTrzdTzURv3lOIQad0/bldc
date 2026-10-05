# BLDC Firmware Agent Instructions

- Read [CONTRIBUTING](CONTRIBUTING) for contribution and C style conventions. Keep changes compatible with supported board configurations, avoid dynamic allocation where practical, and build without warnings. On STM32F4, use `float` and the corresponding `*f` math functions.
- Follow more specific instructions in the directory being changed, including [ChibiOS instructions](ChibiOS_21.11.5/AGENTS.md). Do not apply this repository's general style over a component-specific guide.
- Use the Makefile targets documented in [README.md](README.md): `make` lists firmware boards, `make fw_<board>` builds one board, and `make all_ut_run` runs registered unit tests. Firmware builds require the ARM toolchain.
- On Windows, run Linux firmware commands in the configured WSL environment and verify the intended distribution/toolchain before installing or changing build prerequisites. Keep builds scoped to the requested board.
- Do not flash hardware or initiate motor operation unless the user explicitly requests it.

# Karpathy Guidelines

Behavioral guidelines to reduce common LLM coding mistakes.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.
