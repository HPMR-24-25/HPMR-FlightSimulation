function x_dot = TargetDynamicModel(x, t)

target_ECEF = [x(1); x(2); x(3)];

lla = ecef2lla(target_ECEF', 'WGS84');
lat = lla(1);
lon = lla(2);
alt = lla(3);

R_ET = [
        -sind(lat)*cosd(lon), -sind(lon), -cosd(lat)*cosd(lon);
        -sind(lat)*sind(lon),  cosd(lon), -cosd(lat)*sind(lon);
         cosd(lat),            0,         -sind(lat)
    ];

V_m = sqrt(x(4)^2+x(5)^2+x(6)^2);

% can change psi to 0 for straight line
% can change psi to t for circle
psi = 0;
V = R_ET*[V_m*sin(psi); V_m*cos(psi); 0];

x_dot = [V; 0; 0; 0];