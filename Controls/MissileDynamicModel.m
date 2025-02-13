function [x_dot, accel_ecef] = MissileDynamicModel(x, t, canardInput, AeroModel, MotorModel, const, kins, inds)
%% MissileDynamicModel - Nonlinear dynamic model of missile
% Returns the discrete state derivative of a generic missile model
% Inputs:
%   X - Current state of the vehicle
     % [qw, qx, qy, qz, px, py, pz, vx, vy, vz, m]
%   U - Control Inputs to the vehicle
%   AeroModel - Aerodynamic Model of the vehicle holding aerodynamic data

    %% Calculate LLA to get atmospheric quantities

    r_ecef = [x(inds.px_ecef); x(inds.py_ecef); x(inds.pz_ecef)];

    quat = [x(inds.qw), x(inds.qx), x(inds.qy), x(inds.qz)];

    lla = ecef2lla(r_ecef', 'WGS84');
    lat = lla(1);
    lon = lla(2);
    alt = lla(3);

    atmo = AtmosphericModel(alt);
    
    rho_alt = atmo.getDensity();

    a = sqrt(const.gamma_air*const.R_air*atmo.getTemperature()); % [m/s] Speed of sound at state

    v_b_ecef = [x(inds.vx_ecef); x(inds.vy_ecef); x(inds.vz_ecef)]; % [m/s] Velocity vector in ECEF

    % Relative airspeed
    V_wind_ECEF = WindModel(x, inds);
    % V_wind_ECEF = [0; 0; 0];

    v_ecef = v_b_ecef - V_wind_ECEF;

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

    beta = atan2(v_hat_B(2), v_hat_B(1));
    

    %% Missile Body Drag

    % Dynamic Pressure
    q_inf = 0.5 * rho_alt * (norm(v_ecef)^2);

    C_D = AeroModel.CdLookup(M, AoA);

    if(isnan(C_D)) 
        C_D = AeroModel.CdLookup(M + 0.1, 0);
    end

    D_B = q_inf * C_D * kins.S * (-v_hat_B);

    D_ECEF = R_EB * D_B;

    %% Missile Body Lift
    % L_B = q_inf * AeroModel.ClLookup(M, AoA) * kins.S * cross(v_hat_B, [1; 0; 0]);
    L_B = [0; 0; 0];

    L_ECEF = R_EB * L_B; % [N] Lift Force in ECEF

    %% Gravity Model
    [gx_E, gy_E, gz_E] = xyz2grav(x(inds.px_ecef), x(inds.py_ecef), x(inds.pz_ecef));
    
    %% Motor Thrust
    F_T = MotorModel.thrustPolar(t);
    T_B = [F_T; 0; 0];

    V_exit = MotorModel.Isp * const.g_e;

    m_dot = norm(T_B) / V_exit;
    % m_dot = MotorModel.m_dotPolar(t);

    T_ECEF = R_EB * T_B;

    if(norm(T_B) > 0)
        a = 0;
    end

    %% Canard Forces and Moments
    % Canard-induced lift forces and moments
    L_c_1 = q_inf * kins.canard.S * AeroModel.canard.CL_delta * canardInput.d1; 
    L_c_2 = q_inf * kins.canard.S * AeroModel.canard.CL_delta * canardInput.d2;
    L_c_3 = q_inf * kins.canard.S * AeroModel.canard.CL_delta * canardInput.d3;
    L_c_4 = q_inf * kins.canard.S * AeroModel.canard.CL_delta * canardInput.d4;

    M_1_y = kins.canard.x_cp * L_c_1;
    M_2_y = -kins.canard.x_cp * L_c_2;
    M_3_z = kins.canard.x_cp * L_c_3;
    M_4_z = -kins.canard.x_cp * L_c_4;

    M_1_x = kins.canard.z_cp_13 * L_c_1;
    M_2_x = kins.canard.y_cp_24 * L_c_2;
    M_3_x = kins.canard.z_cp_13 * L_c_3;
    M_4_x = kins.canard.y_cp_24 * L_c_4;

    F_x_B = 0;
    F_y_B = L_c_1 + L_c_3;
    F_z_B = L_c_2 + L_c_4;
    F_c_B = [F_x_B; F_y_B; F_z_B];

    F_c_ECEF = R_EB * F_c_B;

    % Compute wind force coefficient (Cy_wind) based on sideslip angle (beta)
    Cy_wind = 0.3 * beta;
    
    % Apply threshold to avoid wind force at low velocity
    if norm(v_ecef) < 15
        Cy_wind = 0;
    end
    
    % Compute wind-induced force in body frame
    F_wind_B = q_inf * Cy_wind * kins.S * [-v_hat_B(2); 0; v_hat_B(1)];
    
    % Compute the wind-induced moment using the cross product (correct approach)
    M_wind_B = cross([kins.x_cp; 0; 0], F_wind_B);

    %% Damping Moments
    % Damping moments (proportional to angular velocities)
    %% I*omega_ib_dot + omega_ib x I = Sum_Moments
    M_damp_x = -AeroModel.damping.Cd_x * q_inf * kins.S * kins.x_cp * x(inds.w_ib_x);
    M_damp_y = -AeroModel.damping.Cd_y * q_inf * kins.S * kins.x_cp * x(inds.w_ib_y);
    M_damp_z = -AeroModel.damping.Cd_z * q_inf * kins.S * kins.x_cp * x(inds.w_ib_z);

    %% Total Moments
    % M_x_b = M_1_x + M_2_x + M_3_x + M_4_x + M_damp_x + M_wind_B(1); % Roll moment with damping
    % M_y_b = M_1_y + M_2_y + M_damp_y + M_wind_B(2); % Pitch moment with damping
    % M_z_b = M_3_z + M_4_z + M_damp_z + M_wind_B(3); % Yaw moment with damping
    % M_x_b = M_1_x + M_2_x + M_3_x + M_4_x + M_damp_x; % Roll moment with damping
    % M_y_b = M_1_y + M_2_y + M_damp_y; % Pitch moment with damping
    % M_z_b = M_3_z + M_4_z + M_damp_z; % Yaw moment with damping

    C_p = (kins.diameter/2) + (kins.canard.height/2);

    M_x_b = AeroModel.canard.CL_delta * q_inf * kins.canard.S * (canardInput.d1 + canardInput.d3 - canardInput.d2 - canardInput.d4) * C_p + M_damp_x;
    M_y_b = AeroModel.canard.CL_delta * q_inf * kins.canard.S * (canardInput.d1 - canardInput.d3) * kins.x_cp + M_damp_y;
    M_z_b = AeroModel.canard.CL_delta * q_inf * kins.canard.S * (canardInput.d4 - canardInput.d2) * kins.x_cp + M_damp_z;

    %% Angular Accelerations
    % dw_ib_x = M_x_b / kins.I_x;
    % dw_ib_y = M_y_b / kins.I_y;
    % dw_ib_z = M_z_b / kins.I_z;
    dw_ib = kins.I \ (cross(x(inds.w_ib), (kins.I * x(inds.w_ib))) + [M_x_b; M_y_b; M_z_b]);

    %% Quaternion Update
    q_dot = 0.5 * [
        -x(2), -x(3), -x(4);
         x(1), -x(4),  x(3);
         x(4),  x(1), -x(2);
        -x(3),  x(2),  x(1);
    ] * [x(inds.w_ib_x); x(inds.w_ib_y); x(inds.w_ib_z)];

    %% Position Dynamics
    vx_dot = (D_ECEF(1) + L_ECEF(1) + T_ECEF(1) + F_c_ECEF(1)) / x(inds.mass) + gx_E;
    vy_dot = (D_ECEF(2) + L_ECEF(2) + T_ECEF(2) + F_c_ECEF(2)) / x(inds.mass) + gy_E;
    vz_dot = (D_ECEF(3) + L_ECEF(3) + T_ECEF(3) + F_c_ECEF(3)) / x(inds.mass) + gz_E;

    %% State Derivative Vector
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
        dw_ib(1);
        dw_ib(2);
        dw_ib(3);
        -m_dot;
    ];

    accel_ecef = [
        vx_dot;
        vy_dot;
        vz_dot;
    ];

end