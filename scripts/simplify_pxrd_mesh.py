#!/usr/bin/env python3
"""
Simplify the PXRD STL mesh for use with MoveIt2 collision checking.

Usage:
    python3 simplify_pxrd_mesh.py <input.stl> <output.stl> [target_faces]

Default target_faces: 30000 — preserves topology (incl. door cavity) while
running collision checks at interactive speeds.
"""
import sys
import os
import pymeshlab

if len(sys.argv) < 3:
    print(__doc__)
    sys.exit(1)

input_path = sys.argv[1]
output_path = sys.argv[2]
target_faces = int(sys.argv[3]) if len(sys.argv) > 3 else 30000

print(f"Loading: {input_path}")
ms = pymeshlab.MeshSet()
ms.load_new_mesh(input_path)

m = ms.current_mesh()
print(f"Input: {m.vertex_number()} vertices, {m.face_number()} faces")
bbox = m.bounding_box()
print(f"Bounding box (mm): {bbox.dim_x():.1f} x {bbox.dim_y():.1f} x {bbox.dim_z():.1f}")

print(f"Simplifying to ~{target_faces} faces...")
ms.meshing_decimation_quadric_edge_collapse(
    targetfacenum=target_faces,
    preservenormal=True,
    preservetopology=True,
    preserveboundary=True,
    qualitythr=0.3,
    autoclean=True,
)

m = ms.current_mesh()
print(f"Output: {m.vertex_number()} vertices, {m.face_number()} faces")

os.makedirs(os.path.dirname(output_path), exist_ok=True)
ms.save_current_mesh(output_path, binary=True)
print(f"Saved: {output_path}")
print(f"Size: {os.path.getsize(output_path) / 1024:.1f} KB")
