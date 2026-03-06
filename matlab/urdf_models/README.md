# URDF Models

`jetpack_humanoid_simplified.urdf` is a control abstraction for MATLAB tooling.

It is **not** a full MJCF-to-URDF conversion of the PNDbotics Adam Lite humanoid. Instead, it keeps the quantities that matter for hover and jetpack control work:

- total mass,
- rigid-body inertia,
- body-centered base link,
- four fixed thruster mount frames.

This is enough for quick MATLAB Online inspection or Robotics System Toolbox import, while the actual test suite in this folder uses the plain `.m` dynamics model.
