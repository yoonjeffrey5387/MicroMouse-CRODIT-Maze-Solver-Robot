# Micromouse — Flood-Fill Maze Solver

An ESP32 micromouse that explores a 16×16 maze and drives to the center along the shortest known route. Built in 2025 with UABC's **CRODIT** robotics team.

> The firmware here is a clean, simplified rewrite of the original 2025 sketch, kept short so the algorithm is easy to read.

## How it works
1. **Sense** — three IR distance sensors (left, front, right) detect the walls around the current cell.
2. **Map** — each wall is stored as a bit in a 16×16 grid, on both sides of the wall.
3. **Flood fill** — a breadth-first search from the four center cells gives every cell its distance to the goal through the walls known so far.
4. **Move** — the mouse turns toward the open neighbor with the lowest distance (going straight on ties) and drives one cell using wheel encoders.
5. Repeat until it reaches the center. New walls simply trigger a new flood, so the route corrects itself as the maze is discovered.

```
dist map (start of run)        after finding walls
 14 13 12 ...                   14 13 12 ...
  .  .  .  0  0  .              .  .  .  0  0  .
  .  .  .  0  0  .              .  .  .  0  0  .
```

## Hardware
- ESP32
- 3× IR distance sensors (left, front, right)
- 2× DC motors with quadrature encoders, H-bridge driver

## Code
[`firmware/micromouse_floodfill/micromouse_floodfill.ino`](firmware/micromouse_floodfill/micromouse_floodfill.ino). Calibrate `WALL_THRESHOLD`, `TICKS_PER_CELL` and `TICKS_PER_TURN` on your maze and motors.
