function reference = jh_reference_struct(position, velocity, acceleration, attitude, angularRate)
reference.position = position;
reference.velocity = velocity;
reference.acceleration = acceleration;
reference.roll = attitude(1);
reference.pitch = attitude(2);
reference.yaw = attitude(3);
reference.angularRate = angularRate;
end
