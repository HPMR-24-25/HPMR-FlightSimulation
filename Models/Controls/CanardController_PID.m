function canardInput = CanardController_PID(x, accel_cmd_ecef, canard, P, I, D, dt, kins, inds, AeroModel)

    %% for rotation matrix
    r_ecef = [x(inds.px_ecef); x(inds.py_ecef); x(inds.pz_ecef)];
    quat = [x(inds.qw), x(inds.qx), x(inds.qy), x(inds.qz)];
    w = [x(inds.w_ib_x), x(inds.w_ib_y), x(inds.w_ib_z)];
    lla = ecef2lla(r_ecef', 'WGS84');
    lat = lla(1);
    lon = lla(2);
    alt = lla(3);

    R_ET = [
        -sind(lat)*cosd(lon), -sind(lon), -cosd(lat)*cosd(lon);
        -sind(lat)*sind(lon),  cosd(lon), -cosd(lat)*sind(lon);
         cosd(lat),            0,         -sind(lat)
    ];

    R_TB = quat2rotm(quat);

    R_EB = R_ET' * R_TB;

    %% Acceleration to Canard
    accel_B = R_EB'*accel_cmd_ecef;

    AtmosphericModel(alt);

    d = kins.diameter;
    r = kins.x_cp;
    rho_inf = AtmosphericModel.rho_sl;
    v_inf = norm(x(inds.vel));
    S = kins.S; 

    CL_delta = AeroModel.canard.CL_delta;

    q_inf = 0.5 * rho_inf * v_inf^2;

    H = CL_delta * q_inf * S;

    C_p = (kins.diameter/2) + (kins.canard.height/2);


    A = [1 -1 1 -1;
         1 0 -1 0;
         0 -1 0 1];

    b = [0*kins.I_x/(C_p*q_inf * kins.canard.S * AeroModel.canard.CL_delta*v_inf)      *(accel_B(1)+v_inf*(kins.I_z-kins.I_y)*w(2)*w(3)/kins.I_x);
         kins.I_y/(kins.x_cp*q_inf * kins.canard.S * AeroModel.canard.CL_delta*v_inf)*(accel_B(2)+v_inf*(kins.I_x-kins.I_z)*w(1)*w(3)/kins.I_y);
         kins.I_z/(kins.x_cp*q_inf * kins.canard.S * AeroModel.canard.CL_delta*v_inf)*(accel_B(3)+v_inf*(kins.I_y-kins.I_x)*w(2)*w(1)/kins.I_z)];

    canard_des = pinv(A) * b;

    canard1 = canard_des(1);
    canard2 = canard_des(2);
    canard3 = canard_des(3);
    canard4 = canard_des(4);

    canard_Buff = canard';
    canard1_Buff = canard_Buff(:, 1);
    canard2_Buff = canard_Buff(:, 2);
    canard3_Buff = canard_Buff(:, 3);
    canard4_Buff = canard_Buff(:, 4);

    % Canrd 1 Controller
    err_1 = canard1 - canard1_Buff(2);
    err_1_d = ((canard1 - canard1_Buff(2)) - (canard1 - canard1_Buff(1))) / dt;
    err_1_int = ((canard1 - canard1_Buff(2)) - (canard1 - canard1_Buff(1))) * dt;

    canard1_cmd = err_1 * P + err_1_int * I + err_1_d * D;

    % Canard 2 Controller
    err_2 = canard2 - canard2_Buff(2);
    err_2_d = ((canard2 - canard2_Buff(2)) - (canard2 - canard2_Buff(1))) / dt;
    err_2_int = ((canard2 - canard2_Buff(2)) - (canard2 - canard2_Buff(1))) * dt;

    canard2_cmd = err_2 * P + err_2_int * I + err_2_d * D;

    % Canard 3 Controller
    err_3 = canard3 - canard3_Buff(2);
    err_3_d = ((canard3 - canard3_Buff(2)) - (canard3 - canard3_Buff(1))) / dt;
    err_3_int = ((canard3 - canard3_Buff(2)) - (canard3 - canard3_Buff(1))) * dt;

    canard3_cmd = err_3 * P + err_3_int * I + err_3_d * D;

    % Canard 4 Controller
    err_4 = canard4 - canard4_Buff(2);
    err_4_d = ((canard4 - canard4_Buff(2)) - (canard4 - canard4_Buff(1))) / dt;
    err_4_int = ((canard4 - canard4_Buff(2)) - (canard4 - canard4_Buff(1))) * dt;

    canard4_cmd = err_4 * P + err_4_int * I + err_4_d * D;

    canardInput.d1 = canard1_cmd;
    canardInput.d2 = canard2_cmd;
    canardInput.d3 = canard3_cmd;
    canardInput.d4 = canard4_cmd;
end