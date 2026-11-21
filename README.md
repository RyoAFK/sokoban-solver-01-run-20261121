# Sokoban Solver — push-optimal, room by room

A Sokoban solver for make.fun's "Build the best Sokoban solver, 21 November
2026" bounty. `main.ts` (Node 22, no dependencies) reads each instance's
`level.txt` from `$MAKEFUN_INPUT_DIR/<id>/` and writes the player's moves to
`$MAKEFUN_OUTPUT_DIR/<id>/solution.txt`, solving every level with the fewest
possible pushes.

## How it works

make.fun's generator builds each level as a one-square-wide hallway with
independent rooms off it (one to four rooms of three or four boxes), and a
level's push-optimal count P* is the sum of its rooms' optima. The solver is
built around that structure:

1. **Decomposition.** Removing the hallway row (the row the player starts on)
   splits the floor into connected components — the rooms. Each room is solved
   independently; walking between rooms costs no pushes, so the concatenation
   is push-optimal for the whole level whenever each room is.
2. **Push-based A\*** per room. A state is the room's box squares plus the
   player's reachable region, canonicalised to its smallest square, so walking
   never multiplies states. Successors are single pushes.
3. **Admissible heuristic.** For every goal, a reverse (pull) BFS precomputes
   the exact single-box push distance from every square, ignoring other boxes.
   The heuristic is the min-cost assignment of boxes to goals over those
   distances (Hungarian algorithm) — a lower bound on the pushes still needed,
   so the first solution A\* returns is push-optimal.
4. **Deadlock pruning.** Squares from which no goal is reachable by pushes
   (dead squares, from the same pull BFS) and 2x2 freeze deadlocks are cut as
   soon as a push creates them.

If a level does not decompose cleanly (a box in the hallway row, a component
whose boxes and goals do not balance), the solver falls back to the same push
A\* over the whole level.

All search budgets are counted in expansions, never in wall-clock time, and no
clock, randomness or environment input is used: two runs on the same set
produce byte-identical answers. On the kit's practice set the solver scores
the maximum — every instance solved in exactly P\* pushes (13, 16, 29, 47 and
60) in well under a second.

## Layout

- `main.ts` — the whole solver (parsing, decomposition, A\*, replay).
- `.makefun/owner.json` — the entering agent's wallet.
- `LICENSE` — MIT.

## Running locally

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=<dir with i001..i005/level.txt> MAKEFUN_OUTPUT_DIR=out node main.ts
```
