function kins = HPMR_ModelRocketKinematics()

% kins.x_cp = 9 / 39.37; % [m] Longitudinal center of pressure distance
kins.x_cg = 65.485 / 39.37;
kins.x_cp = (70.294 / 39.37) - kins.x_cg;
% kins.x_cp = 0.12;
kins.I_x = 0.044;  % [kg/m^2]
kins.I_y = 11.834; % [kg/m^2]
kins.I_z = 11.834; % [kg/m^2]
kins.I = diag([kins.I_x, kins.I_y, kins.I_z]);
kins.diameter = 4 / 39.37; % [m] Diameter (in -> m)
kins.S = pi * (kins.diameter^2 / 4);

% Rocket Geometry
kins.len = 99.831 / 39.37; % [m] Missile Length (in -> m)
kins.x_nose = 20 / 39.37;
% Mass Properties
kins.m_0 = 212 / 35.274; % [kg] Dry Mass (oz -> kg)

% Canard Properties
canard.rootChord = 5.5 / 39.37; % [m] (in -> m)
canard.tipChord  = 1.5 / 39.37; % [m] (in -> m)
canard.height    = 4   / 39.37; % [m] (in -> m)
canard.S         = (canard.rootChord + canard.tipChord / 2) * canard.height;

kins.x_canard = kins.x_nose + (10.471 / 39.37) + (canard.rootChord / 2);

canard.x_cp = kins.x_cg - kins.x_canard;

% canard.x_cp      = 27 / 39.37; % [m] (in -> m)
canard.y_cp_13   = 0;
canard.z_cp_13   = canard.height / 2;
canard.y_cp_24   = canard.height / 2;
canard.z_cp_13   = 0;
canard.maxActuationRate = 0.5; % [rad/s]
canard.maxActuation     = deg2rad(15); % [deg]

kins.canard = canard;

end