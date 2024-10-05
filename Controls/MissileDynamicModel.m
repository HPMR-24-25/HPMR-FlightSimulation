function x_dot = MissileDynamicModel(x, t, canardInput, AeroModel, MotorModel, const, kins, inds)
%% MissileDynamicModel - Nonlinear dynamic model of missile
% Returns the discrete state derivative of a generic missile model
% Inputs:
%   X - Current state of the vehicle
     % [qw, qx, qy, qz, px, py, pz, vx, vy, vz, m]
%   U - Control Inputs to the vehicle
%   AeroModel - Aerodynamic Model of the vehicle holding aerodynamic data

    %% Calcualte LLA to get atmospheric quantities

    r_ecef = [x(inds.px_ecef); x(inds.py_ecef); x(inds.pz_ecef)];

    quat = [x(inds.qw), x(inds.qx), x(inds.qy), x(inds.qz)];

    lla = ecef2lla(r_ecef', 'WGS84');
    lat = lla(1);
    lon = lla(2);
    alt = lla(3);

    atmo = AtmosphericModel(alt);
    
    rho_alt = atmo.getDensity();

    a = sqrt(const.gamma_air*const.R_air*atmo.getTemperature()); % [m/s] Speed of sound at state

    v_ecef = [x(inds.vx_ecef); x(inds.vy_ecef); x(inds.vz_ecef)]; % [m/s] Velocity vector in ECEF

    M = norm(v_ecef) / a; % Mach Number

    %% Rotation Matrix Setup

    R_ET = [
        -sind(lat)*cosd(lon), -sind(lon), -cosd(lat)*cosd(lon);
        -sind(lat)*sind(lon),  cosd(lon), -cosd(lat)*sind(lon);
         cosd(lat),            0,         -sind(lat)
    ];

    R_TB = quat2rotm(quat);

    R_EB = R_ET * R_TB;

    %% Unit Vector Calculation

    v_hat_ecef = v_ecef / norm(v_ecef); % [1] Unit Vector of Velocity in ECEF

    v_hat_B = R_EB' * v_hat_ecef; % [1] Unit Vector of Velocity in Body

    AoA = atan2(v_hat_B(3), v_hat_B(1)); AoA = rad2deg(AoA);
    
    % Dynamic Pressure
    q_inf = 0.5 * rho_alt * norm(v_ecef)^2;

    %% Missile Body Drag

    % D_B = q_inf * AeroModel.CdLookup(M, AoA) * kins.S * -v_hat_B;
    if(M <= 1)
        C_D = 0.08*(1 + exp((-1)*(3*(1 - M))^2))
    else
        C_D = 0.08*(1 + exp((-1)*(M-1)^2));
    end

    D_B = q_inf * C_D * kins.S * (-v_hat_B);

    D_ECEF = R_EB * D_B;

    %% Missile Body Lift
    % L_B = q_inf * AeroModel.C(M, AoA) * kins.S * cross(v_hat_B, [1; 0; 0]);
    L_B = [0; 0; 0];

    L_ECEF = R_EB * L_B; % [N] Lift Force in ECEF

    [gx_E, gy_E, gz_E] = xyz2grav(x(inds.px_ecef), x(inds.py_ecef), x(inds.pz_ecef));
    %% Thrust Calculation
    F_T = MotorModel.thrustPolar(t);
    T_B = [F_T; 0; 0]

    V_exit = MotorModel.Isp * const.g_e;

    m_dot = norm(T_B) / V_exit;
    % m_dot = MotorModel.m_dotPolar(t);

    T_ECEF = R_EB * T_B;

    if(norm(T_B) > 0)
        a = 0;
    end

    % D_T = R_ET' * D_ECEF;

    %% Position Dynamics
    vx_dot = (D_ECEF(1) + L_ECEF(1) + T_ECEF(1)) / x(inds.mass) + gx_E;
    vy_dot = (D_ECEF(2) + L_ECEF(2) + T_ECEF(2)) / x(inds.mass) + gy_E;
    vz_dot = (D_ECEF(3) + L_ECEF(3) + T_ECEF(3)) / x(inds.mass) + gz_E;

    %% Attitude Dynamics
    % phi_dot = 0;
    % % omegay_dot = (-kins.x_cp * norm(L_B)) / kins.I_y;
    % pitch_dot = 0;
    % yaw_dot = 0;
    
    % p = x(11);
    % q = x(12);
    % r = x(13);

    w_ib_x = 0;
    w_ib_y = 0;
    w_ib_z = 0;

    p = w_ib_x;
    q = w_ib_y;
    r = w_ib_z;

    dw_ib_x = 0;
    dw_ib_y = 0;
    dw_ib_z = 0;

    % pitch_dot = M_c_pitch / (kins.I_y*x(inds.mass));
    % yaw_dot   = M_c_yaw / (kins.I_z*x(inds.mass));
    % roll_dot = 0;

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
        dw_ib_x;
        dw_ib_y;
        dw_ib_z;
        -m_dot;
    ];

    if(t >= 4 && t <= 8)
        brk = 0;
    end

end