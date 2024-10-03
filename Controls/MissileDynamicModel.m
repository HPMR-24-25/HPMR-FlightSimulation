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

    R_ET = [
        -sind(lat)*cosd(lon), -sind(lon), -cosd(lat)*cosd(lon);
        -sind(lat)*sind(lon),  cosd(lon), -cosd(lat)*sind(lon);
         cosd(lat),            0,         -sind(lat)
    ];

    R_TB = quat2rotm(quat);

    R_EB = R_ET * R_TB;

    %% Conversions

    a = sqrt(const.gamma_air*const.R_air*atmo.getTemperature()); % [m/s] Speed of sound at state

    v_ecef = [x(inds.vx_ecef); x(inds.vy_ecef); x(inds.vz_ecef)]; % [m/s] Velocity vector in ECEF

    M = norm(v_ecef) / a; % Mach Number

    %% Unit Vector Calculation

    v_hat_ecef = v_ecef / norm(v_ecef); % [1] Unit Vector of Velocity in ECEF

    v_hat_B = R_EB' * v_hat_ecef % [1] Unit Vector of Velocity in Body

    AoA = atan2(v_hat_B(3), v_hat_B(1)); AoA = rad2deg(AoA);
    
    % Dynamic Pressure
    q_inf = 0.5 * rho_alt * norm(v_ecef)^2;

    %% Drag Force
    

    % D_B = q_inf * AeroModel.CdLookup(M, AoA) * kins.S * -v_hat_B;
    if(M <= 1)
        C_D = 0.08*(1 + exp((-1)*(3*(1 - M))^2));
    else
        C_D = 0.08*(1 + exp((-1)*(M-1)^2));
    end

    D_B = q_inf * C_D * kins.S * (-v_hat_B);

    D_ECEF = R_EB * D_B;

    %% Lift Force
    % L_B = q_inf * AeroModel.C(M, AoA) * kins.S * cross(v_hat_B, [1; 0; 0]);
    L_B = [0; 0; 0];

    L_ECEF = R_EB * L_B; % [N] Lift Force in ECEF

    % G_ECEF = R_TE' * g_T; % [m/s^2] Gravity in ECEF
    [gx_E, gy_E, gz_E] = xyz2grav(x(inds.px_ecef), x(inds.py_ecef), x(inds.pz_ecef));

    %% Thrust Calculation
    F_T = MotorModel.thrustPolar(t);
    T_B = [F_T; 0; 0];

    V_exit = MotorModel.Isp * const.g_e;

    % m_dot = norm(T_B) / V_exit;
    m_dot = MotorModel.m_dotPolar(t);

    T_ECEF = R_EB * T_B;

    D_T = R_ET' * D_ECEF;

    %% Position Dynamics
    vx_dot = (D_ECEF(1) + L_ECEF(1) + T_ECEF(1)) / x(inds.mass) + gx_E;
    vy_dot = (D_ECEF(2) + L_ECEF(2) + T_ECEF(2)) / x(inds.mass) + gy_E;
    vz_dot = (D_ECEF(3) + L_ECEF(3) + T_ECEF(3)) / x(inds.mass) + gz_E;

    %% Attitude Dynamics
    phi_dot = 0;
    % omegay_dot = (-kins.x_cp * norm(L_B)) / kins.I_y;
    pitch_dot = 0;
    yaw_dot = 0;
    
    p = x(11);
    q = x(12);
    r = x(13);

    q_dot = [
         0.5*p*x(2) - 0.5*q*x(3) - 0.5*r*x(4);
         0.5*p*x(1) - 0.5*q*x(4) + 0.5*r*x(3);
         0.5*p*x(4) + 0.5*q*x(1) - 0.5*r*x(2);
        -0.5*p*x(3) + 0.5*q*x(2) + 0.5*r*x(1);
    ];

    x_dot = [
        q_dot(1);
        q_dot(2);
        q_dot(3);
        q_dot(4);
        x(inds.vx_ecef);
        x(inds.vy_ecef);
        x(inds.vz_ecef);
        vx_dot;
        vy_dot;
        vz_dot;
        phi_dot;
        pitch_dot;
        yaw_dot;
        -m_dot;
    ];

end