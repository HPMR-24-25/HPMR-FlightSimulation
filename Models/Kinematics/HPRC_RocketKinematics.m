function kins = HPRC_RocketKinematics()

kins.x_cp = -0.1778; % [m] Longitudinal center of pressure distance
kins.I_x = 0.046;  % [kg/m^2]
kins.I_y = 10.436; % [kg/m^2]
kins.I_z = 10.436; % [kg/m^2]
kins.I = diag([kins.I_x, kins.I_y, kins.I_z]);
kins.diameter = 4 / 39.37; % [m] Diameter (in -> m)
kins.S = pi * (kins.diameter^2 / 4);
kins.len = 93 / 39.37; % [m] Missile Length (in -> m)

% Mass Properties
kins.m_0 = 442 / 35.274; % [kg] Dry Mass (oz -> kg)

% Airbrakes Properties

end