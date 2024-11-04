function kins = HPMR_MissileKinematics()

kins.x_cp = -0.1778; % [m] Longitudinal center of pressure distance
kins.I_x = 5;
kins.I_y = 26;
kins.I_z = 26;
kins.I = diag([kins.I_x, kins.I_y, kins.I_z]);
kins.S = 8.17e-3; % [m^2] Missile Frontal Reference Area
kins.len = 2.5; % [m] Missile Length

% Mass Properties
kins.m_i = 30.028; % [kg] Initial Mass
kins.m_0 = 13.245; % [kg] Dry Mass

kins.canard.S = 0.1; % [m^2] Canard Surface Area
kins.canard.x_cp = 1; % [m] Distance from CM to canard CP

end