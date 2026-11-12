*This activity has been created as part of the 42 curriculum by masalvad.*

# Fly-in

## Description

Fly-in is a drone routing simulation written in Python. The goal is to move a fleet of drones from a start hub to an end hub through a weighted graph, in the fewest possible simulation turns.

Each hub has a zone type that affects movement cost:
- **normal** — 1 turn
- **priority** — 1 turn, preferred by the pathfinding algorithm
- **restricted** — 2 turns (drone is in transit and invisible for 1 turn)
- **blocked** — inaccessible

Capacity constraints apply both to hubs (`max_drones`) and connections (`max_link_capacity`). The algorithm distributes drones across multiple paths to avoid bottlenecks.

## Instructions

### Requirements

```bash
make install
```

Installs: `pydantic`, `mypy`, `flake8`.

### Run

```bash
make run MAP=maps/easy/01_linear_path.txt
```

Output format per turn: `D<id>-<hub> D<id>-<hub> ...`

### Visual mode

```bash
make debug MAP=maps/medium/02_circular_loop.txt
```

Launches the simulation under `pdb`. For colored output instead:

```bash
python3 main.py maps/medium/02_circular_loop.txt --visual
```

Colors reflect each hub's `color=` attribute from the map file. Zone-type fallback: restricted → purple, priority → cyan, normal → blue.

### Lint

```bash
make lint          # flake8 + mypy with strict flags
make lint-strict   # mypy --strict
```

### Clean

```bash
make clean
```

## Algorithm

### Pathfinding — Dijkstra

`Graph.find_best_path(start, excluded)` runs Dijkstra on the adjacency list with weighted edges:

| Zone type | Cost |
|---|---|
| restricted | 2.0 |
| normal | 1.0 |
| priority | 0.5 |

The `excluded` set blocks hubs that are full or already reserved by another drone this turn. This forces drones onto alternative paths, distributing traffic naturally.

### Dynamic per-turn routing

Paths are **recalculated every turn** for each drone. This lets the algorithm adapt to congestion as it develops rather than committing to a fixed plan upfront.

### Multi-path distribution

Drones are processed sequentially each turn. As each drone reserves its next hub, that hub is added to the `excluded` set for subsequent drones, steering them toward different paths automatically.

### Restricted zone transit

Moving into a restricted zone costs 2 turns. The drone sets `in_transit = True` and records `transit_dest`. On the next turn, transit arrivals are resolved **first** (before other movements), so the drone appears at the destination hub on turn T+1.

### Complexity

- Dijkstra per drone per turn: O((V + E) log V)
- With D drones and T turns: O(D × T × (V + E) log V)
- No path caching — each turn gets a fresh view of congestion state

## Visual Representation

The `--visual` flag enables ANSI-colored terminal output:

- **Turn number** — bold yellow
- **Drone ID** — unique color per drone (cycles through 8 colors)
- **Hub name** — colored by the hub's `color=` attribute from the map file, with zone-type fallback (purple=restricted, cyan=priority, blue=normal, green=start/end)
- **Completion summary** — total turns in bold green

All colors are resolved at runtime from the map file metadata, so custom maps with their own color schemes render correctly.

## Performance Benchmarks

| Map | Turns | Target | Status |
|---|---|---|---|
| easy/01 linear path (2 drones) | 6 | ≤ 6 | ✓ |
| easy/02 simple fork (3 drones) | — | ≤ 6 | — (file modified) |
| easy/03 basic capacity (4 drones) | 6 | ≤ 8 | ✓ |
| medium/01 dead end trap (5 drones) | 16 | ≤ 15 | ~1 off |
| medium/02 circular loop (6 drones) | 19 | ≤ 20 | ✓ |
| medium/03 priority puzzle (4 drones) | 11 | ≤ 12 | ✓ |
| hard/01 maze nightmare (8 drones) | 21 | ≤ 45 | ✓ |
| hard/02 capacity hell (12 drones) | 18 | ≤ 60 | ✓ |
| hard/03 ultimate challenge (15 drones) | 54 | ≤ 35 | in progress |
| challenger (25 drones) | 171 | ≤ 45 | optional |

## Resources

### References

- [Dijkstra's algorithm — Wikipedia](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)
- [Python `heapq` documentation](https://docs.python.org/3/library/heapq.html)
- [Pydantic v2 documentation](https://docs.pydantic.dev/latest/)
- [PEP 257 — Docstring conventions](https://peps.python.org/pep-0257/)
- [mypy documentation](https://mypy.readthedocs.io/en/stable/)

### AI usage

Claude (Anthropic) was used throughout this activity to:

- **Algorithm design** — guided step-by-step implementation of Dijkstra with weighted zone costs, discussion of BFS vs Dijkstra trade-offs
- **Simulation architecture** — design of the two-pass turn loop (transit arrivals first, then movements), the `reserved` set for multi-path distribution, and the `in_transit` mechanism for restricted zones
- **Debugging** — identifying the `in_transit` bug where drones appeared at restricted hubs one turn early, and the `check_link_capacity` NameError
- **Code quality** — flake8 and mypy compliance, docstring formatting, type annotations

All generated code was reviewed, tested against all provided maps, and understood before being kept.
