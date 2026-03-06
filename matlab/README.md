# Jetpack Humanoid MATLAB Suite

This folder contains a MATLAB Online-friendly test and simulation package for the PNDbotics Adam Lite jetpack humanoid model in:

- `../../pndbotics_adam_lite/jetpack/adam_lite_original_bag.xml`

The relevant source assets are available in:

- `../../pndbotics_adam_lite/jetpack/assets`
- `../../pndbotics_adam_lite/assets`

The package does **not** depend on MuJoCo, Simulink, or desktop-only tooling. It uses a control-oriented rigid-body model derived from the MJCF:

- Total mass, inertia, and thruster locations were taken from the jetpack humanoid model.
- Rear thrusters and front boosters keep their original thrust limits and gimbal limits.
- The simulator includes thrust saturation, gimbal saturation, simple rate limits, and a basic ground clamp for takeoff and landing.

## Run In MATLAB Online

1. Open this `matlab` folder in MATLAB Online and run:

   ```matlab
   results = run_jetpack_humanoid_suite;
   ```

2. If you stay in the project root instead, run:

   ```matlab
   addpath(fullfile(pwd, 'matlab'))
   results = run_jetpack_humanoid_suite;
   ```

3. For a path-reset run from the project root:

   ```matlab
   setup_control_jetpack_path(pwd)
   results = run_jetpack_humanoid_suite(false);
   ```

4. Or use the root helper:

   ```matlab
   results = run_suite_from_root(false);
   ```

5. For a non-plot run:

   ```matlab
   results = run_jetpack_humanoid_suite(false);
   ```

6. The suite will:
   - compute capability metrics,
   - run baseline hover, takeoff, landing, maneuverability, and yaw-authority tests,
   - run disturbance rejection, figure-eight tracking, payload sensitivity, actuator-stress, single-thruster degradation, and gain-sweep evaluations,
   - open plots,
   - save `results/jetpack_humanoid_suite_results.mat`.

## What The Tests Answer

- Can the current jetpack layout generate enough thrust to hover and take off?
- Do the gimbaled thrusters provide enough control authority for pitch, roll, yaw, and lateral translation near hover?
- Can the vehicle hold hover, climb, descend, and land in a first-order rigid-body simulation?
- Are the current thruster positions good enough for general hover operations?
- How well does the controller recover from an external push or torque pulse?
- How much tracking error appears on a continuous figure-eight trajectory?
- How sensitive is hover stability to heavier payloads or shifted center of gravity?
- What happens when commands push thrust or gimbal actuators near their limits?
- Does hover remain feasible with one degraded rear thruster?
- How do controller gains trade overshoot, settling, and tilt during takeoff?

## Real-World Limits

This package is useful for early feasibility screening, not certification. It does **not** model:

- plume interaction with the humanoid body,
- thermal limits,
- propulsion efficiency or fuel mass flow,
- structural stress on mounts,
- landing leg contact loads,
- actuator latency from a real embedded controller,
- aerodynamic drag beyond a small linear damping term.

If the suite fails, the layout is likely wrong. If it passes, the layout is only **first-order plausible** and still needs a higher-fidelity dynamics and hardware review.
