#!/usr/bin/env python3
"""
Strip the 'world' link and 'fixed' joint from a Fairino URDF, leaving the
arm chain rooted at base_link. The output can be xacro:include'd into a
custom URDF that mounts base_link wherever needed.

Usage:
    python3 strip_fairino_world.py <input.urdf> <output.urdf>
"""
import sys
import re

if len(sys.argv) != 3:
    print(__doc__)
    sys.exit(1)

input_path, output_path = sys.argv[1], sys.argv[2]

with open(input_path, "r") as f:
    content = f.read()

# Remove the <link name="world"/> line
content = re.sub(
    r'\s*<link\s+name="world"\s*/>\s*\n',
    "\n",
    content,
    count=1,
)

# Remove the entire <joint name="fixed">...</joint> block
content = re.sub(
    r'\s*<joint\s+name="fixed"[^>]*>.*?</joint>\s*\n',
    "\n",
    content,
    count=1,
    flags=re.DOTALL,
)

with open(output_path, "w") as f:
    f.write(content)

print(f"Stripped world link from {input_path}")
print(f"Wrote {output_path}")

# Verify
with open(output_path, "r") as f:
    verify = f.read()
has_world_link = '<link name="world"' in verify
has_fixed_joint = '<joint name="fixed"' in verify
print(f"  Has 'world' link?  {has_world_link}")
print(f"  Has 'fixed' joint? {has_fixed_joint}")
if not has_world_link and not has_fixed_joint:
    print("  OK")
