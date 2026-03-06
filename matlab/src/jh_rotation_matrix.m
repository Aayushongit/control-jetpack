function rotation = jh_rotation_matrix(eulerAngles)
phi = eulerAngles(1);
theta = eulerAngles(2);
psi = eulerAngles(3);

cphi = cos(phi);
sphi = sin(phi);
ctheta = cos(theta);
stheta = sin(theta);
cpsi = cos(psi);
spsi = sin(psi);

rotation = [
    cpsi * ctheta, cpsi * stheta * sphi - spsi * cphi, cpsi * stheta * cphi + spsi * sphi;
    spsi * ctheta, spsi * stheta * sphi + cpsi * cphi, spsi * stheta * cphi - cpsi * sphi;
    -stheta, ctheta * sphi, ctheta * cphi
];
end
