function x_dot = MissileDynamicModel(x, t, AeroModel, MotorModel, const, kins, inds)
%% MissileDynamicModel - Nonlinear dynamic model of missile
% Returns the discrete state derivative of a generic missile model
% Inputs:
%   X - Current state of the vehicle
     % [qw, qx, qy, qz, px, py, pz, vx, vy, vz, m]
%   U - Control Inputs to the vehicle
%   AeroModel - Aerodynamic Model of the vehicle holding aerodynamic data

    r_ecef = [x(inds.px_ecef); x(inds.py_ecef); x(inds.pz_ecef)];

    quat = [x(inds.qw), x(inds.qx), x(inds.qy), x(inds.qz)];

    lla = ecef2lla(r_ecef', 'WGS84');
    lat = lla(1);
    lon = lla(2);
    alt = lla(3);

    atmo = AtmosphericModel(alt);

    % r_ecef = [x(inds.px_ecef); x(inds.py_ecef); x(inds.pz_ecef)];
    % 
    % r_ned = R_TE * r_ecef;
    % 
    % atmo = AtmosphericModel(-r_ned(3));

    rho_alt = atmo.getDensity();

    %% Rotation Matrix Setup

    R_TE = [
        -sind(lat)*cosd(lon), -sind(lon), -cosd(lat)*cosd(lon);
        -sind(lat)*sind(lon),  cosd(lon), -cosd(lat)*sind(lon);
         cosd(lat),            0,         -sind(lat)
    ];

    R_TB = quat2rotm(quat);

    R_EB = R_TE' * R_TB;

    %% Conversions

    a = sqrt(const.gamma_air*const.R_air*atmo.getTemperature()); % [m/s] Speed of sound at state

    v_ecef = [x(inds.vx_ecef); x(inds.vy_ecef); x(inds.vz_ecef)]; % [m/s] Velocity vector in ECEF

    M = norm(v_ecef) / a; % Mach Number

    %% Unit Vector Calculation

    v_hat_ecef = v_ecef / norm(v_ecef); % [1] Unit Vector of Velocity in ECEF

    v_hat_B = R_EB' * v_hat_ecef; % [1] Unit Vector of Velocity in Body

    AoA = atan2(v_hat_B(3), v_hat_B(1)); AoA = rad2deg(AoA);

    
    % Dynamic Pressure
    q_inf = 0.5 * rho_alt * norm(v_ecef)^2;

    %% Drag Force

    D_B = q_inf * AeroModel.CdLookup(M, AoA) * kins.S * v_hat_B;

    D_ECEF = R_EB * D_B;

    %% Lift Force
    % L_B = q_inf * AeroModel.C(M, AoA) * kins.S * cross(v_hat_B, [1; 0; 0]);
    L_B = 0;

    L_ECEF = R_EB * L_B; % [N] Lift Force in ECEF

    g_T = [0; 0; -const.g_e]; % **TODO** APPLY GRAVITY MODEL

    G_ECEF = R_TE' * g_T; % [m/s^2] Gravity in ECEF

    %% Thrust Calculation
    T_B = [MotorModel.thrustPolar(t); 0; 0];

    V_exit = MotorModel.Isp * const.g_e;

    m_dot = norm(T_B) / V_exit;

    T_ECEF = R_EB * T_B;

    %% Position Dynamics
    vx_dot = (D_ECEF(1) + L_ECEF(1) + T_ECEF(1)) / x(inds.mass) + G_ECEF(1);
    vy_dot = (D_ECEF(2) + L_ECEF(2) + T_ECEF(2)) / x(inds.mass) + G_ECEF(2);
    vz_dot = (D_ECEF(3) + L_ECEF(3) + T_ECEF(3)) / x(inds.mass) + G_ECEF(3);

    %% Attitude Dynamics
    omegax_dot = 0;
    omegay_dot = (-kins.x_cp * norm(L_B)) / kins.I_y;
    omegaz_dot = 0;

    

    x_dot = [
        0;
        0;
        0;
        0;
        x(inds.vx_ecef);
        x(inds.vy_ecef);
        x(inds.vz_ecef);
        vx_dot;
        vy_dot;
        vz_dot;
        omegax_dot;
        omegay_dot;
        omegaz_dot;
        -m_dot;
    ];

end