# Robot Cell Engineering Playbook

Distilled, reusable methods from building the **unchained** (FR16, hardware-proven)
and **pxrd** (FR10, enclosed-instrument) cells. Written so the NEXT cell — any
robot + instrument + labware combination — can be built by following recipes
instead of rediscovering lessons. Examples cite real numbers from these cells.

---

## 1. Cell-build recipe (the template)

Order of operations that worked twice:

1. **Instrument first, as ground truth.** Export instrument CAD from SolidWorks
   with a *custom coordinate system at the process point* (sample position,
   deck datum). One `instrument` link, world joint `rpy="1.5708 0 0"` converts
   CAD Y-up → ROS Z-up, `device_height` arg lifts it to the floor.
2. **Place everything by origin-to-origin SolidWorks measurements.** Measure
   in CAD: instrument-origin → pedestal-origin (e.g. pxrd: 127.63, −624.6,
   410.12 mm). CAD (x,y,z) maps directly into the child-joint xyz when the
   parent is the instrument's CAD frame. CAVEAT: this works for *assemblies
   sharing the frame*; an independently-exported PART may have a permuted
   frame — then place by feature instead (see §2.3).
3. **Robot mount**: pedestal top → base_link with `rpy="-1.5708 ${yaw} 0"`.
   **The mount rotation goes in the PITCH slot** — a Y-up parent's vertical
   axis is Y; URDF yaw (about Z) tips the robot over. Symptom if wrong: arm
   lies sideways. Companion rules: `HOME j1 = -mount_yaw`, grasp yaw
   compensation `= -mount_yaw`.
4. **Deck markers** as tiny visual-only child links at each pick/place point,
   Y-up frames matching the instrument. ALL motion code addresses decks by
   TF lookup — never hardcoded coordinates.
5. **Peripherals** (tables, stands) via world- or parent-frame args with
   sane defaults; everything overridable from the launch CLI.
6. **One combined xacro** consumed by MoveItConfigsBuilder; every arg
   declared in the launch AND forwarded in the xacro mappings — a mapping
   you forget to forward is *silently* ignored (cost us a day: the plate
   "wasn't there" because the launch never passed the flag).

## 2. Mesh pipeline

### 2.1 Conventions
- Source STLs stay **mm**; URDF references use `scale="0.001"`. Meshes
  converted to meters must NOT also carry the scale. Pick one per mesh and
  note it in the file/README.
- Decimate collision meshes with `fast_simplification` (pip): 300k→50k tris
  is fine for an instrument. NEVER convex-hull an enclosure — the cavity IS
  the workspace.
- Re-emit binary STL with recomputed normals (cross-product per face).

### 2.2 Verifying a mesh you didn't make
Slice it numerically before trusting it (bbox, per-axis histograms of
10 mm slices). This is how paddle heights, pocket layouts, and enclosure
apertures were measured. Vertex sampling lies about large flat faces
(vertices only at corners) — slice-emptiness ≠ material-emptiness.

### 2.3 Placement when frames don't match
An independently-exported part with a permuted frame: recenter the mesh on
a *feature* (e.g. well-body center), then dial rotation with launch args —
**rotating about the feature keeps the feature pinned** while you fix
orientation (rot_x/rot_y/rot_z in the parent frame, one axis at a time).
Detect needed pre-flips numerically: apply the intended transform, check a
known-up feature ends up (paddle must end above the base). 

### 2.4 CAD revision diffs
New instrument export vs old: voxel-diff (20 mm voxels; dilate the old set
by 1 voxel to suppress alignment noise) → the added/removed geometry bbox.
Found a new 9-10 obstruction this way in minutes. If bboxes match but
origins differ, the delta of the bboxes gives the pure translation to bake
into the converted mesh (keeps every downstream reference unchanged).

## 3. Grasp & choreography design (decide from geometry, not hope)

1. **Map the reachable corridor** of an enclosed target from the collision
   mesh: occupancy slices around the process point (ASCII maps are enough).
   Identify: aperture (open heights), walls, roof, interior shelf heights.
2. **Test the wrist-stack against the aperture**: a top-down grasp is a
   ~350 mm vertical stack above the TCP. If that stack must cross a wall
   plane, top-down is GEOMETRICALLY IMPOSSIBLE — no planner tuning will fix
   it. The pxrd proof: forearm/wrist1 contacts on every IK branch.
3. **Side-pinch = the human-hand pattern** for slot-like enclosures:
   gripper horizontal, jaw axis vertical, whole chain at slot height.
   Requires labware with a GRAB FEATURE (paddle/tab) designed for it:
   - flat parallel pinch faces, thickness ≪ jaw stroke;
   - standoff from the payload center = how far the wrist stays outside;
   - pinch height driven by the *measured* fingertip envelope (AG-145 tips
     extend far below the pads — the first paddle failed for exactly this).
   Iterate the labware in sim with MOCK meshes (translate the feature) before
   cutting real CAD.
4. **Choreography primitives** (compose per cell):
   - vertical lift to carry height FIRST → later swings become level;
   - staged ladder to any far target: posture(j2/j3) → j1 swing → wrist
     pre-orient → settle. START-AWARE ordering: from low/home, posture first
     (low j1-swings drag the gripper through the pedestal); from beside an
     instrument, j1 first (re-posturing there reaches into it);
   - Cartesian insert/retract along a fixed axis for enclosures — the
     collision-checked fraction is your depth gauge and diagnostic;
   - micro-lift before extracting; partial jaw openings near floors;
   - offset-pinch + deposit compensation when clearances force grabbing
     off-center (a force gripper doesn't care).
5. **Pin IK branches.** Same TCP pose has mirror wrist families with totally
   different collision outcomes. Probe once for the valid family, save it as
   a seed hint (STAGE_HINTS pattern), seed the IK there every run.
   Determinism > planner luck; taught approaches are the industrial norm.

## 4. Collision honesty (non-negotiable)

- **Never mask to make a motion pass.** Two real incidents: (a) inherited
  SRDF "slot clearance" disables hid gripper-vs-instrument — fingertips were
  passing through a shelf in "successful" runs; (b) ARM_LINKS in the plate's
  touch_links hid plate-vs-forearm — the user SAW the plate clip the arm.
  Whitelist only *physically legitimate* contacts, at the narrowest scope
  (e.g. plate-resting-on-instrument via `extra_touch` on that one attach).
- Oversized "safety" obstacles can FORBID physically-proven motions
  (unchained deck-1 box). Model obstacles at measured size + explicit margin
  — or better, let the instrument scan carry them.
- The human eye is a collision detector the checker can't replace: run
  visual passes; when the user says "that looks like it hits", believe them
  and measure.

## 5. Debugging toolkit (build these into every cell)

| Tool/technique | What it answers |
|---|---|
| `probe_pose` (IK w/o collision + check_state_validity) | is a pose unreachable, or colliding — and WHICH link pair |
| Cartesian A/B (avoid_collisions True vs False) | real collision vs solver truncation at fraction<1 |
| move_group launch log: `Found a contact between X and Y` | why a computed path was rejected — read, don't guess |
| binary-search a pose parameter (height, depth) | exact clearance thresholds in 2-3 probes |
| `/joint_states` name[]+position[] paired | ground-truth joint capture (order is scrambled; a mis-paired j4 sign cost a day) |
| escape primitive: Cartesian lift w/ avoid_collisions=False | ONLY way out of start-state-in-collision grazes |
| `recover` tool: envelope detect → corridor back-out (count only executed steps — partial Cartesian plans DON'T execute) → multi-attempt home | mandatory before hardware: aborts inside an instrument must be self-recovering |
| zombie sims: `ps -o stat` (Tl/Z), not pgrep | Ctrl-Z'd launches look alive and eat hours |

Budget gates (max total/per-joint/per-step travel) on every executed plan:
the last line of defense against a crazy plan reaching hardware.

## 6. Sim → hardware transfer

- One cell-agnostic BRIDGE (`fr_bridge`): FollowJointTrajectory in,
  vendor-SDK out, /joint_states back, /stop_motion service. The C++ vendor
  plugin was buggy; the Python bridge is the proven path.
- Same command ± `--hardware`. Real speed = bridge `movej_vel_pct`;
  `--vel` is planning-only pacing. First runs: 3-5%, pendant override 25%.
- ROS_DOMAIN_ID per cell (42/43) → sims coexist with zero interference.
- Gripper truths (DH AG-145): force-grip beats position (drive nearly shut,
  force limit stops on the object; stall = grip); feedback is transiently
  wrong after commands — debounce on position; calibrate stroke with a
  caliper (`--gripper-mm`).
- Tilt-critical payloads (open vials): hard orientation path-constraint on
  every carried phase (±2° feasible; exact 0 is unplannable), low velocity,
  and remember Cartesian phases hold orientation by construction — the
  constraint matters on the OMPL joint-space phases.

## 7. Process discipline

- **Checkpoint commit before any restructure**; keep an on-disk legacy copy
  the first time a working system is reorganized.
- **Record the verbatim working terminal commands** (launch args + flags)
  in the docs at every milestone — they are the reproducibility contract.
- Canonical launchers (`run_<cell>.sh`) with validated flags baked in.
- Phase-log orchestration: every motion phase prints [OK/FAIL] + metrics
  (travel, fraction, duration); the phase summary is the debugging record.
- Never modify YAML/params that ran on hardware — duplicate.
- When a "successful" run contradicts strict checking, suspect masking
  first, geometry drift second, planner luck third — in that order.

## 8. Multi-agent build pattern (for big integrations)

Parallelize by FILE OWNERSHIP, brief with MEASURED NUMBERS:
one agent per non-overlapping artifact set (meshes / scene URDF+launch /
motion code), each brief carrying exact dimensions, frames, and the
verification commands to self-check (xacro parse, ast.parse). Integrator
(you) holds the context, does the build, tunes, and runs the validation
matrix. Used for the 2026-09 unchained refit (new instrument scan + stand +
plate types + tilt constraints built in one parallel pass).

## 9. Refit lessons (2026-09 unchained refit — new scan + stand + tall racks)

- **Approach heights must be payload-relative, never absolute.** A fixed
  `hover 0.15` above the deck was BELOW the grasp height of a 77 mm rack —
  the "approach above" pose dangled splayed-open fingertips at obstacle
  height (~1 mm interference with a 72 mm rail; deterministic contact), and
  the low carry grazed the same rail crossing another deck column. Rule:
  hover = grasp_offset + pick_lift + clearance. One bug, three symptoms —
  all invisible until strict collision checking.
- **Thin obstacles defeat coarse motion validation.** Symptom signature:
  "Computed path is not valid. Invalid states at index [k]" at the SAME
  index for every planner and every attempt, plan never executes. Cause:
  OMPL validates motions at longest_valid_segment resolution; thin sheets
  slip between checks and only post-resampling validation catches them.
  Fix: set `longest_valid_segment_fraction` ~0.002 (inject via your launch
  after MoveItConfigsBuilder — vendored configs stay untouched).
- **MoveIt purge ordering trap: detaching an AttachedCollisionObject
  RE-INSERTS it into the world.** A purge that removes from the world first
  and detaches second leaves ghosts forever while "reporting success" — and
  back-to-back jobs poison themselves (run N+1 inherits a ghost at its own
  source). Detach from all candidate links FIRST, world-remove LAST; also
  purge every enumerable spawned-object id, not just the current run's.
  The tell we ignored for weeks: a benign-looking
  "apply_planning_scene returned failure" in every log.
- **Model updates must preserve small safety-critical features.** Decimation
  destroyed an 18-vertex obstruction sheet (the exact geometry under test);
  re-extract such features from the full-res mesh by region (centroid
  filter) and append to the collision mesh. Verify by occupancy query, not
  by eyeball.
- **Replacing synthetic obstacle boxes with the real scan** resolves
  box-oversize contradictions AND requires re-auditing every SRDF disable
  that referenced the old hull (a carved scan makes gripper-vs-instrument
  checking honest — verify carving by corridor occupancy before removing
  whitelists).
- **Parallel sim validation**: one agent per scratch ROS domain, each
  launches its own headless copy of the scene, runs a test slice, greps its
  own move_group log for "Found a contact between", tears down by verifying
  /proc/<pid>/environ domain before killing. Findings from agents running
  pre-fix code must be re-screened after any fix (results can share one
  root cause).
