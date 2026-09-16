# CAD sources

Source CAD for the robot cells. These are the ORIGINALS exported from
SolidWorks; the meshes actually used at runtime are decimated copies inside
each ROS package (`ros2_ws/src/<cell>/meshes/`).

## pxrd/
| File | What |
|---|---|
| `SmartLab_96well.STL` | Rigaku SmartLab PXRD enclosure, origin = sample position (custom SW coordinate system) |
| `tabbed_plate_v2.STL` | **Current** 96-well plate with raised grab paddle (was `check_v2.STL`) — validated in sim |
| `tabbed_plate_v1.STL` | First tabbed plate iteration (paddle too low for the AG-145 fingertips) |
| `plate_ext_KN_v2.STL` | Plate extension design working file |
| `vention_pedestal_945mm.STL` | Tall Vention pedestal (355606), origin = top plate |

## unchained/
| File | What |
|---|---|
| `unchained_junior_v2.STL` | Unchained Junior instrument (current) |
| `unchained_junior_v1.STL` / `_origin.STL` | Earlier exports |
| `vention_pedestal_585mm.STL/.STEP` | Short Vention pedestal (288080 v8.2), origin = top plate |
| `585_pedestal.STL` | Early pedestal sketch |

## Regenerating runtime meshes
Runtime meshes were produced with `fast_simplification` (pip) — decimate,
recenter (plate: on the well-body center; needs preflip-X180 for v2), write
binary STL into the package `meshes/` dir. See memory/docs for exact recipes.
