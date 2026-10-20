#!/usr/bin/env python3
"""
Generate the v3 tabbed-plate meshes for the PXRD cell from cad/pxrd/check.STL
(2026-10-20 drop: low OVERHANG tab — paddle 15 mm thick, pinch center 26.5 mm
above the plate bottom, 155.7 mm out from the well center, so it overhangs the
table edge / SmartLab stage edge and the AG-145 lower fingertip hangs in free
air below deck level).

Frame conversion (differs from v2's pipeline!):
  v2 raw was exported wells-up (+Y) and needed a preflip-X180.
  v3 raw (check.STL) is exported wells-DOWN (-Y = physically up), which is
  ALREADY the processed-mesh convention (deck_up = -mesh_y). So: NO rotation.
  - mesh_x = raw_x          (well grid already centered at X=0)
  - mesh_y = raw_y + 0.9mm  (plate bottom raw +2.0 -> mesh +2.9, same as v2,
                             so the baked TABBED_PLATE_POSE puts the plate
                             bottom at deck +5.6 mm exactly like v2)
  - mesh_z = raw_z          (well grid already centered at Z=0)
  - scale mm -> m

Outputs (into ros2_ws/src/pxrd_cell/meshes/):
  the_plate_v3_visual.stl     full-resolution
  the_plate_v3_collision.stl  quadric-decimated to ~13k faces (v2 parity)

Measured v3 geometry (mesh frame, meters) -> constants in pick_place.py:
  TAB_CENTER_X  = -0.1557   (flat paddle spans X -0.1730..-0.1384)
  TAB_THICKNESS =  0.015    (raw Y -32..-17 exactly)
  TAB_WIDTH     =  0.060    (paddle Z -25.4..+34.6; 4.6 mm lateral offset,
                             ignored same as v2's 4.1 mm)
  TAB_PINCH_Y   =  0.0321   (deck-local: 8.5 pose offset + 23.6 mesh height)
"""
import struct
import sys
import os

RAW = os.path.join(os.path.dirname(__file__), "..", "cad", "pxrd", "check.STL")
OUTDIR = os.path.join(os.path.dirname(__file__), "..",
                      "ros2_ws", "src", "pxrd_cell", "meshes")
Y_SHIFT_MM = 0.9
TARGET_FACES = 13000


def read_stl(path):
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        data = f.read()
    tris = []
    for i in range(n):
        rec = struct.unpack_from("<12f", data, i * 50)
        tris.append(rec)
    return tris


def write_stl(path, tris, header=b"pxrd v3 tabbed plate (meters)"):
    with open(path, "wb") as f:
        f.write(header.ljust(80, b" ")[:80])
        f.write(struct.pack("<I", len(tris)))
        for rec in tris:
            f.write(struct.pack("<12f", *rec))
            f.write(b"\x00\x00")


def transform(rec):
    # rec = (nx,ny,nz, 9 vertex floats). Normals unchanged (pure translate+scale).
    out = list(rec[:3])
    for v in range(3):
        x, y, z = rec[3 + v * 3: 6 + v * 3]
        out += [x * 1e-3, (y + Y_SHIFT_MM) * 1e-3, z * 1e-3]
    return out


def main():
    tris = [transform(r) for r in read_stl(RAW)]
    vis = os.path.join(OUTDIR, "the_plate_v3_visual.stl")
    col = os.path.join(OUTDIR, "the_plate_v3_collision.stl")
    write_stl(vis, tris)
    print(f"visual: {vis} ({len(tris)} tris)")

    import pymeshlab
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(vis)
    ms.meshing_decimation_quadric_edge_collapse(
        targetfacenum=TARGET_FACES, preservenormal=True,
        preservetopology=True, preserveboundary=True,
        qualitythr=0.3, autoclean=True)
    ms.save_current_mesh(col, binary=True)
    m = ms.current_mesh()
    print(f"collision: {col} ({m.face_number()} faces)")

    # verify bboxes
    for p in (vis, col):
        t = read_stl(p)
        xs, ys, zs = [], [], []
        for rec in t:
            for v in range(3):
                xs.append(rec[3 + v * 3]); ys.append(rec[4 + v * 3]); zs.append(rec[5 + v * 3])
        print(f"{os.path.basename(p)}: X {min(xs):+.4f}..{max(xs):+.4f} "
              f"Y {min(ys):+.4f}..{max(ys):+.4f} Z {min(zs):+.4f}..{max(zs):+.4f}")


if __name__ == "__main__":
    sys.exit(main())
