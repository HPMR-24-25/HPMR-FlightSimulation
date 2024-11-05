function kins = HPMR_ModelRocketKinematics()

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

% Canard Properties
canard.rootChord = 5.5 / 39.37; % [m] (in -> m)
canard.tipChord  = 1.5 / 39.37; % [m] (in -> m)
canard.height    = 4   / 39.37; % [m] (in -> m)
canard.S         = (canard.rootChord + canard.tipChord / 2) * canard.height;
canard.x_cp      = 23  / 39.37; % [m] (in -> m)
canard.y_cp_13   = 0;
canard.z_cp_13   = canard.height / 2;
canard.y_cp_24   = canard.height / 2;
canard.z_cp_13   = 0;

kins.canard = canard;

end