# makefun-sokoban-solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/theo1603/makefun-sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/theo1603/makefun-sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver: `main.ts` splits each level into its hallway
rooms (a box in the hallway can never reach a goal, so rooms are independent)
and solves every room with A* over pushes — states are boxes plus the player's
reachable region, the heuristic is the cheapest box-to-goal assignment under
reverse-BFS push distances, dead squares and frozen 2x2 blocks prune hopeless
states. Weighted A* and greedy best-first fall back when the optimal search
runs out of budget. Deterministic: budgeted by expansions, never by the clock.

## Run it

```sh
MAKEFUN_INPUT_DIR=./input MAKEFUN_OUTPUT_DIR=./output node main.ts
```

Reads `$MAKEFUN_INPUT_DIR/<id>/level.txt` for every instance, writes
`$MAKEFUN_OUTPUT_DIR/<id>/solution.txt`. Scores 1,000,000 (push-optimal on all
5 instances) on the kit's practice set.

---

<sub>Entered on [make.fun](https://make.fun/?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
