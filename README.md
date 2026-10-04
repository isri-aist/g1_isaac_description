# g1_isaac_description

This package provides the Unitree G1 humanoid models for `mc_isaac` (Isaac Sim simulation of
[mc_rtc](https://jrl-umi3218.github.io/mc_rtc/) controllers), like `g1_mj_description` does for
[mc_mujoco](https://github.com/rohanpsingh/mc_mujoco).

It is a data-only package (no compilation, no mc_rtc dependency): USD models and one mc_isaac description file per
mc_rtc robot module.

It installs:

- `share/mc_isaac/<module>.yaml`: one description per mc_rtc robot module of `mc_g1` that has a USD
  - `g1_29dof_no_hands.yaml`
  - `g1_29dof_revo2.yaml`
- `share/g1_isaac_description/usd/`: the USD models referenced by these descriptions

## Models

| mc_rtc module (`MainRobot` alias) | Description | USD | DOF |
|---|---|---|---|
| `g1_29dof_no_hands` (`G1_no_hands`) | G1 29 DOF, tool connectors instead of hands | `g1_no_hands.usd` | 29 |
| `g1_29dof_revo2` (`G1_Revo2`) | G1 29 DOF with two BrainCo Revo2 hands | `g1_revo2.usd` (+ `g1_no_hands.usd`, `revo2_left.usd`, `revo2_right.usd`) | 51 (29 body + 2 × 6 actuated hand joints + 2 × 5 mimic) |

`G1` / `G1_23dof` (`g1_23dof`) and `G1_29dof` (`g1_29dof`) have no USD yet.

Both models have a **floating base** (`fixed: false`, articulation root on `pelvis`): the robot stands on the ground
plane and mc_isaac sends the measured state back to mc_rtc:

- `FloatingBase` body sensor (on `pelvis`, the root body): position, orientation, linear and angular velocity of the
  root in the world frame
- `Accelerometer` body sensor (on `torso_link`): simulated IMU, orientation, angular velocity and linear acceleration
  (proper acceleration, gravity included) in the sensor frame. The acceleration is computed by finite differences of
  the sensor velocity over the physics steps of a controller step.

The hand distal joints (`(left|right)_(thumb|index|middle|ring|pinky)_distal_joint`) are PhysX mimic joints
(`PhysxMimicJointAPI` in the USD): PhysX makes them follow their proximal joint, they have no drive and the commands
mc_rtc sends for them are ignored.

Collisions: every link has collision shapes and self-collisions are enabled, like `g1_mj_description` (MuJoCo
collides all geom pairs except parent/child bodies). PhysX ignores the pairs of links connected by a joint; the pair
`torso_link` / `waist_yaw_link`, whose hulls overlap at rest, is filtered in the USD (`FilteredPairsAPI`).

mc_isaac looks descriptions up by **robot module name** (`robot.module().name` in mc_rtc), then robot name, then the
`MainRobot` value for the main robot, so `MainRobot: G1_Revo2` works.

## Description files

Each `<module>.yaml` contains:

```yaml
usd: <absolute path of the installed USD>
extra_files: [g1_no_hands.usd, revo2_left.usd, revo2_right.usd]  # g1_29dof_revo2 only
fixed: false
self_collisions: true  # overrides the articulation setting of the USD (false in the IsaacLab asset)
drives:            # PhysX joint drives, regex groups (fullmatch) over the Isaac joint names
  - {joints: "left_hip_pitch_joint", stiffness: 200.0, damping: 4.0}
  ...
```

`g1_revo2.usd` only assembles the other three USDs (references relative to its folder): `extra_files` makes mc_isaac
upload them with it to the Isaac server (with the same relative layout).

Units and meaning of the drives are the same as IsaacLab `ImplicitActuatorCfg` (applied by the mc_isaac server through
the PhysX tensor API, in radians). Keys not given (`max_effort`, `max_velocity`, `armature`) keep the values stored in
the USD. Joints that no group matches keep the gains stored in the USD (mc_isaac prints a warning, except for mimic
joints).

Gains (from `g1_mj_description` `pdgains/<module>/PDgains_sim.dat`):

| Joints | Stiffness (N.m/rad) | Damping (N.m.s/rad) |
|---|---|---|
| `left_hip_pitch_joint` / `right_hip_pitch_joint` | 200 | 4 / 3 |
| `(left\|right)_hip_(roll\|yaw)_joint` | 100 | 2 |
| `(left\|right)_knee_joint` | 210 | 6 |
| `left_ankle_pitch_joint` / `right_ankle_pitch_joint` | 150 | 4 / 3 |
| `(left\|right)_ankle_roll_joint` | 150 | 3 |
| `waist_yaw_joint` | 300 | 10 |
| `waist_(roll\|pitch)_joint` | 300 | 3 |
| `(left\|right)_shoulder_(pitch\|roll)_joint` | 100 | 2 |
| `(left\|right)_shoulder_yaw_joint`, `(left\|right)_elbow_joint` | 50 | 2 |
| `(left\|right)_wrist_(roll\|pitch\|yaw)_joint` | 2 | 2 |
| `(left\|right)_thumb_metacarpal_joint` (Revo2) | 20 | 1.0 |
| `(left\|right)_thumb_proximal_joint` (Revo2) | 40 | 1.2 |
| `(left\|right)_(index\|middle\|ring)_proximal_joint` (Revo2) | 35 | 1.0 |
| `(left\|right)_pinky_proximal_joint` (Revo2) | 30 | 0.8 |

The left/right asymmetries come from the mc_mujoco gain files and are kept as is.

With `simulation: torque_control: true` in the IsaacSim plugin configuration, the gains are set to 0 and the mc_rtc
torques are applied as joint efforts instead.

## Build and install

```bash
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=<prefix>
make install
```

Installed in the mc_isaac (or mc_rtc) prefix, the descriptions are found without configuration; otherwise add
`<prefix>/share/mc_isaac` to `description_paths` in the IsaacSim plugin configuration
(`~/.config/mc_rtc/plugins/IsaacSim.yaml`).

Option `-DSRC_MODE=ON` makes the descriptions point to the USD files of the source tree (no USD copy at install).

## Model generation

The USD files come from the IsaacLab assets of the G1 and Revo2 robots, converted from their URDFs with the Isaac Sim
URDF importer (Revo2 mimic joints as `PhysxMimicJointAPI`). `revo2_left.usd` and `revo2_right.usd` are the same files
as in `revo2_isaac_description`, duplicated so that this package stays self-contained.

Two changes were made to the IsaacLab assets (with the `pxr` USD API, in the Isaac Sim python):

- `g1_revo2.usd`: the IsaacLab asset mounted the hands rotated by 90° about the hand base z axis with respect to the
  mc_rtc URDF. The fixed joints `<side>_hand_base_joint` (`/g1/joints`) now use the URDF mount
  `<side>_tool_attach_joint` of `g1_description` (same as `g1_mj_description`): `localPos0` (0.022, 0, 0) in
  `<side>_tool_connector_link`, `localRot0` (wxyz) (0.5, -0.5, 0.5, -0.5) left / (0.5, 0.5, 0.5, 0.5) right
  (rpy (-π/2, 0, -π/2) / (π/2, 0, π/2)), `localPos1`/`localRot1` identity; the `/g1/revo2_<side>` prim transforms were
  updated to the same pose so that the scene starts consistent.
- `g1_no_hands.usd`: the IsaacLab asset only had collisions on the pelvis, torso, knees, wrist yaw links and feet. A
  convex hull of the visual mesh (as MuJoCo does) was added to the 22 other links (hidden `mc_isaac_collisions` scope
  under each link, `UsdPhysics.MeshCollisionAPI` approximation `convexHull`), and the `torso_link` / `waist_yaw_link`
  pair, overlapping at rest, is filtered (`UsdPhysics.FilteredPairsAPI`). Masses and inertias are unchanged.

## Adding a variant

1. Add the USD(s) in `usd/` (joint names identical to the mc_rtc URDF; list the files referenced by the main USD in
   `extra_files`, relative to its folder).
2. Add `mc_isaac/<module name>.in.yaml` (`@G1_USD_DIR@` is replaced by the USD folder) and the module name to
   `G1_MODELS` in `CMakeLists.txt`.
3. Rebuild and install.
