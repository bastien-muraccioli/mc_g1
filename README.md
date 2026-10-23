mc_g1
=======

This package provides a RobotModule implementation for the G1 robot.

Dependencies
------------

This package requires:
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- [g1_description](https://github.com/bastien-muraccioli/g1_description)

Optional runtime dependency for composed variants (`*_Revo2`):
- [mc_revo2](https://github.com/isri-aist/mc_revo2)

How to use
------------

Put the following in `mc_rtc.yaml`.
```yaml
MainRobot: G1_mode_15 # use the mode_machine of your robot, see below
Timestep: 0.002
```

Available module names
------------

Pick the model matching the hardware revision of your robot, given by its `mode_machine` (Unitree app: **Device → Data → Robot → Machine Type**; `MCControlG1` from mc_unitree2 also prints it at startup and refuses to run with a mismatching model). See [g1_description](https://github.com/bastien-muraccioli/g1_description) for the differences between revisions.

| `mode_machine` | Module name (short) | Module name (full) | Actuated joints |
|:---:|---|---|:---:|
| 4 | `G1_mode_4` | `G1_23dof_rev_1_0` | 23 |
| 5 | `G1_mode_5`, `G1` | `G1_29dof_rev_1_0` | 29 |
| 6 | `G1_mode_6` | `G1_29dof_lock_waist_rev_1_0` | 27 (waist roll/pitch locked) |
| 10 | `G1_mode_10` | `G1_23dof_mode_10` | 23 |
| 11 | `G1_mode_11` | `G1_29dof_mode_11` | 29 |
| 12 | `G1_mode_12` | `G1_29dof_mode_12` | 27 (waist roll/pitch locked) |
| 13 | `G1_mode_13` | `G1_29dof_mode_13` | 29 |
| 14 | `G1_mode_14` | `G1_29dof_mode_14` | 27 (waist roll/pitch locked) |
| 15 | `G1_mode_15` | `G1_29dof_mode_15` | 29 |
| 16 | `G1_mode_16` | `G1_29dof_mode_16` | 27 (waist roll/pitch locked) |
| 18 | `G1_mode_18` | `G1_29dof_mode_18` | 29 |
| 1 (deprecated) | `G1_mode_1` | `G1_23dof` | 23 |
| 2 (deprecated) | `G1_mode_2` | `G1_29dof` | 29 |

For every 29dof model, two more modules are available (e.g. for mode 15):
- `G1_mode_15_no_hands` / `G1_29dof_mode_15_no_hands`: hands replaced by tool connectors
- `G1_mode_15_Revo2` / `G1_29dof_mode_15_Revo2`: no-hands model with `Revo2_LeftHand` and `Revo2_RightHand` connected (requires [mc_revo2](https://github.com/isri-aist/mc_revo2))

`G1`, `G1_no_hands` and `G1_Revo2` are aliases of `G1_mode_5`, `G1_mode_5_no_hands` and `G1_mode_5_Revo2` (robot names `g1_29dof_rev_1_0`, `g1_29dof_rev_1_0_no_hands`, `g1_29dof_rev_1_0_revo2`). The deprecated 29dof model (mode 2) is available as `G1_mode_2` / `G1_29dof` (and their `_no_hands` / `_Revo2` variants).

The mc_rtc robot name is `g1_<full name in lower case without G1_>`, e.g. `g1_29dof_mode_15`, `g1_29dof_mode_15_no_hands`, `g1_29dof_mode_15_revo2`.

Self-collisions
------------

`minimalSelfCollisions` (also used as `commonSelfCollisions`) contains:
- arms (shoulder yaw, elbow) against the torso and the pelvis
- hands against the torso and the pelvis
- hands against the legs: hip roll link, thigh (`hip_yaw_link`) and shin (`knee_link`) of the same leg, thigh and shin of the other leg (interaction distance 4 cm, safety distance 2 cm). The hand bodies are the rubber hand and the wrist yaw link (29dof), the wrist roll hand (23dof), the wrist yaw link and tool connector (`_no_hands`), plus the palm and finger tips for `_Revo2`
- legs against each other (hips, knees, ankles, feet)

Sensors
------------

The real G1 has two IMUs, both declared as body sensors (poses read from the `imu_in_pelvis_joint` / `imu_in_torso_joint` fixed joints of the URDF when available):

| Body sensor | Parent link | Real robot topic |
|---|---|---|
| `Accelerometer` (default) | `pelvis` | `rt/lowstate` (`imu_state`), main IMU |
| `TorsoAccelerometer` | `torso_link` | `rt/secondary_imu` |
| `FloatingBase` | `pelvis` | ground truth (simulation only) |

In [mc_mujoco](https://github.com/rohanpsingh/mc_mujoco), the MuJoCo model must provide `<sensor name>_gyro` and `<sensor name>_accelerometer` sensors (see [g1_mj_description](https://github.com/bastien-muraccioli/g1_mj_description)).
