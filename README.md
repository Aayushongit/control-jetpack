# Control-Jetpack

`control-jetpack` is a MATLAB Online-friendly control and feasibility study for the PNDbotics Adam Lite jetpack humanoid model.

It is built around the MuJoCo/MJCF source model located in:

- `../pndbotics_adam_lite/jetpack/adam_lite_original_bag.xml`

The original robot assets are stored in:

- `../pndbotics_adam_lite/jetpack/assets` for the jetpack MJCF alongside `adam_lite_original_bag.xml`
- `../pndbotics_adam_lite/assets` as the shared parent asset directory

## Purpose

This project answers the early-stage control and physical-layout questions for the jetpack humanoid:

- Can the current thruster layout support hover?
- Can it take off and land in a controlled way?
- Is there enough control authority for roll, pitch, yaw, and lateral maneuvering?
- Are the thruster positions on the humanoid body reasonable for general hovering operations?
- Is the configuration obviously infeasible, or is it first-order plausible for further development?

This is a feasibility and control-screening project, not a final flight-certification model.

## What Is In This Folder

- [matlab](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab)
  The MATLAB package containing the simulator, controller, test scenarios, plots, and results output.
- [matlab/run_jetpack_humanoid_suite.m](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/run_jetpack_humanoid_suite.m)
  Main entry point for running the complete test suite.
- [matlab/src](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/src)
  Core dynamics, control allocation, parameter, and capability-analysis functions.
- [matlab/tests](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/tests)
  Scenario definitions for hover, takeoff, landing, maneuverability, and yaw-authority testing.
- [matlab/urdf_models](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/urdf_models)
  Simplified URDF abstraction for quick robotics-tooling import and inspection.
- [matlab/results](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/results)
  Saved `.mat` result files from completed runs.

## Model Basis

The MATLAB model is a control-oriented rigid-body abstraction derived from the MJCF jetpack humanoid model.

It uses:

- total vehicle mass,
- body inertia,
- rear jetpack thruster locations,
- front booster locations,
- thrust limits,
- gimbal angle limits,
- simple actuator rate limits.

The simplified URDF in [matlab/urdf_models/jetpack_humanoid_simplified.urdf](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/urdf_models/jetpack_humanoid_simplified.urdf) is not a full MJCF-to-URDF conversion. It is only a compact geometry-and-inertia abstraction for analysis workflows.

## Test Coverage

The suite currently includes:

- `Hover Hold`
  Checks whether the humanoid can stabilize at a fixed altitude and hold attitude.
- `Takeoff`
  Checks controlled climb from ground level into hover.
- `Landing`
  Checks controlled descent, touchdown, and low sink rate near the ground.
- `Maneuverability`
  Checks lateral translation and altitude retention while remaining stable.
- `Yaw Authority`
  Checks whether the gimbaled thrusters can generate usable yaw control without unacceptable drift.

The suite also computes capability metrics such as:

- thrust-to-weight ratio,
- hover throttle fraction,
- maximum net vertical acceleration,
- near-hover control rank,
- approximate force and torque authority,
- approximate lateral acceleration capability.

## Current High-Level Findings

From the current model abstraction:

- The configuration is not obviously impossible.
- The thrust-to-weight ratio is about `2.03`, which is enough for hover and vertical climb margin.
- The gimbaled-thruster control model has full rank `6` near hover in the rigid-body approximation.
- Hover, takeoff, landing, maneuverability, and yaw-authority scenario tests all passed in local Octave validation.
- The current layout appears first-order plausible for hover-capable operation and further control development.

That does not mean the design is ready for real hardware deployment.

## What The Simulation Assumes

The simulator includes:

- 6-DoF rigid-body translational and rotational dynamics,
- thrust saturation,
- gimbal saturation,
- thrust and gimbal rate limiting,
- simple linear translational and rotational damping,
- a basic ground clamp for takeoff and landing scenarios,
- feedback control for position and attitude tracking.

The main simulation loop is implemented in [matlab/src/jh_simulate_scenario.m](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/src/jh_simulate_scenario.m).

## What This Project Does Not Yet Model

The current package does not include:

- detailed humanoid joint motion during flight,
- plume interaction with the torso, legs, or arms,
- thrust losses from cross-flow or body interference,
- structural stress in mounts and backpack frames,
- thermal constraints,
- battery/fuel mass flow and endurance,
- detailed landing-contact mechanics in the feet and legs,
- sensor noise and estimator errors,
- real embedded implementation latency,
- actuator failures or fault tolerance.

So the right interpretation is:

- If this suite fails, the layout is likely poor or under-actuated.
- If this suite passes, the layout is only suitable for the next development stage, not final validation.

## How To Run In MATLAB Online

1. From the project root, either open [matlab](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab) and run:

```matlab
results = run_jetpack_humanoid_suite;
```

2. Or stay in the project root and run:

```matlab
addpath(fullfile(pwd, 'matlab'));
results = run_jetpack_humanoid_suite;
```

3. For a path-reset run from the project root:

```matlab
setup_control_jetpack_path(pwd);
results = run_jetpack_humanoid_suite(false);
```

4. Or use the root helper:

```matlab
results = run_suite_from_root(false);
```

5. For a non-plot run from either location:

```matlab
results = run_jetpack_humanoid_suite(false);
```

6. Results are saved to:

- [matlab/results/jetpack_humanoid_suite_results.mat](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/results/jetpack_humanoid_suite_results.mat)

## Key Files

- [matlab/src/jh_params.m](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/src/jh_params.m)
  Defines mass, inertia, thruster placement, limits, and controller gains.
- [matlab/src/jh_analyze_capability.m](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/src/jh_analyze_capability.m)
  Computes physical capability and control-authority metrics.
- [matlab/src/jh_controller.m](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/src/jh_controller.m)
  Converts reference motion into thruster and gimbal commands.
- [matlab/src/jh_allocate_commands.m](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/src/jh_allocate_commands.m)
  Performs near-hover control allocation.
- [matlab/tests/test_takeoff.m](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/tests/test_takeoff.m)
  Takeoff scenario.
- [matlab/tests/test_landing.m](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/tests/test_landing.m)
  Landing scenario.
- [matlab/tests/test_maneuverability.m](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/tests/test_maneuverability.m)
  Translation and maneuverability scenario.
- [matlab/tests/test_yaw_authority.m](/media/dell/Hard Drive/Summer of code/Robotics/tests-march2026/control-jetpack/matlab/tests/test_yaw_authority.m)
  Yaw-control scenario.

## Recommended Next Steps

- Add a higher-fidelity propulsion model with efficiency and power draw.
- Add disturbance tests for gusts, actuator lag, and partial thrust loss.
- Add structural checks for mount loads and torso-backpack interfaces.
- Add leg-contact and foot-impact modeling for realistic landing safety.
- If needed, build a second-stage model that couples humanoid joint posture with jetpack control.
