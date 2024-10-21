function kins = HPMR_MissileKinematics()

kins.x_cp = -0.1778; % [m] Longitudinal center of pressure distance
kins.I_y = 26;
kins.I_x = 26;
kins.I_z = 5;
kins.S = 0.81; % [m^2] Frontal Reference Area

% Mass Properties
kins.m_i = 30.028; % [kg] Initial Mass
kins.m_0 = 13.245; % [kg] Dry Mass

kins.canard.S = 0.1; % [m^2] Canard Surface Area
kins.canard.x_cp = 1; % [m] Distance from CM to canard CP

end