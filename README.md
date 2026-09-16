# Ks_FR — Fairino robot cells for lab automation

Two ROS2/MoveIt2 pick-and-place cells that load 96-well plates into lab
instruments with Fairino cobots. Same code runs in simulation and on
hardware.

| Cell | Robot → Instrument | Status |
|---|---|---|
| unchained | FR16 → Unchained Junior | hardware-proven |
| pxrd | FR10 → Rigaku SmartLab (enclosed) | sim-validated round trip |

## Quick start

```bash
cd ros2_ws && colcon build --symlink-install && cd ..
./run_pxrd.sh sim          # PXRD sim + rviz     (second terminal: ./run_pxrd.sh load)
./run_unchained.sh sim     # Unchained sim + rviz
```

The launchers bake in the **latest validated terminal values** — PXRD:
`tall_pedestal:=true pedestal_off_z:=0.55 pedestal_off_x:=0
pedestal_off_y:=-0.2 table_x:=-0.5 table_y:=-0.9 table_h:=0.5`; Unchained
hardware: `table_x:=-0.3 table_y:=-1.7 table_h:=1`. The full verbatim
commands live at the top of each cell doc.

Docs: [CLAUDE.md](CLAUDE.md) (overview + rules) ·
[docs/pxrd-cell.md](docs/pxrd-cell.md) ·
[docs/unchained-cell.md](docs/unchained-cell.md) ·
[cad/README.md](cad/README.md)

Requires ROS2 Humble, MoveIt2, and the vendored Fairino packages in
`ros2_ws/src/ros2_fr_gazebo` (firmware v3.8.6).
