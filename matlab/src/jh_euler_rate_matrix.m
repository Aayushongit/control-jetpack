function transform = jh_euler_rate_matrix(eulerAngles)
phi = eulerAngles(1);
theta = eulerAngles(2);

cosTheta = cos(theta);
if abs(cosTheta) < 1.0e-4
    cosTheta = sign(cosTheta) * 1.0e-4 + (cosTheta == 0.0) * 1.0e-4;
end

transform = [
    1.0, sin(phi) * tan(theta), cos(phi) * tan(theta);
    0.0, cos(phi), -sin(phi);
    0.0, sin(phi) / cosTheta, cos(phi) / cosTheta
];
end
