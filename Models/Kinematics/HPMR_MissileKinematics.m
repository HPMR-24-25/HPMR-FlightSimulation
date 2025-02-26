function kins = HPMR_MissileKinematics()

kins.x_cp = 9 / 39.37; % [m] Longitudinal center of pressure distance
% kins.I_x = 0.044;  % [kg/m^2]
% kins.I_y = 11.834; % [kg/m^2]
% kins.I_z = 11.834; % [kg/m^2]
kins.I_x = 0.02069; % [kg/m^2]
kins.I_y = 3.69984; % [kg/m^2]
kins.I_z = 3.69984; % [kg/m^2]
% kins.I = diag([kins.I_x, kins.I_y, kins.I_z]);
kins.diameter = 4 / 39.37; % [m] Diameter (in -> m)
kins.S = pi * (kins.diameter^2 / 4);
kins.len = 98 / 39.37; % [m] Missile Length (in -> m)

% Mass Properties
kins.m_0 = 471 / 35.274; % [kg] Dry Mass (oz -> kg)

% Canard Properties
canard.rootChord = 4.5 / 39.37; % [m] (in -> m)
canard.tipChord  = 0.5 / 39.37; % [m] (in -> m)
canard.height    = 3   / 39.37; % [m] (in -> m)
canard.S         = (canard.rootChord + canard.tipChord / 2) * canard.height;
canard.x_cp      = 27 / 39.37; % [m] (in -> m)
canard.y_cp_13   = 0;
canard.z_cp_13   = canard.height / 2;
canard.y_cp_24   = canard.height / 2;
canard.z_cp_13   = 0;
canard.maxActuationRate = 0.2; % [rad/s]
canard.maxActuation     = deg2rad(10); % [deg]

kins.canard = canard;

end