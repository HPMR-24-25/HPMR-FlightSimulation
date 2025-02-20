function canardInput = AccelerationController_PID(x, accel_ecef, desiredAcceleration, P, I, D, dt, kins, inds, AeroModel)

    %% for rotation matrix
    r_ecef = [x(inds.px_ecef); x(inds.py_ecef); x(inds.pz_ecef)];
    quat = [x(inds.qw), x(inds.qx), x(inds.qy), x(inds.qz)];
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

    R_EB = R_ET * R_TB;

    %% PID
    accelBuff = accel_ecef';
    axBuff = accelBuff(:, 1);
    ayBuff = accelBuff(:, 2);
    azBuff = accelBuff(:, 3);

    ax = desiredAcceleration(1);
    ay = desiredAcceleration(2);
    az = desiredAcceleration(3);

    % Roll Controller
    err_x = ax - axBuff(2);
    err_x_d = ((ax - axBuff(2)) - (ax - axBuff(1))) / dt;
    err_x_int = ((ax - axBuff(2)) - (ax - axBuff(1))) * dt;

    accel_x = err_x * P + err_x_int * I + err_x_d * D;

    % Pitch Controller
    err_y = ay - ayBuff(2);
    err_y_d = ((ay - ayBuff(2)) - (ay - ayBuff(1))) / dt;
    err_y_int = ((ay - ayBuff(2)) - (ay - ayBuff(1))) * dt;

    accel_y = err_y * P + err_y_int * I + err_y_d * D;

    % Yaw Controller
    err_z = az - azBuff(2);
    err_z_d = ((az - azBuff(2)) - (az - azBuff(1))) / dt;
    err_z_int = ((az - azBuff(2)) - (az - azBuff(1))) * dt;

    accel_z = err_z * P + err_z_int * I + err_z_d * D;

    accel = [accel_x; accel_y; accel_z];

    accel_B = R_EB'*accel;

    AtmosphericModel(alt);

    d = kins.diameter;
    r = kins.x_cp;
    rho_inf = AtmosphericModel.rho_sl;
    v_inf = norm(x(inds.vel));
    S = kins.S; 

    CL_delta = AeroModel.canard.CL_delta;

    q_inf = 0.5 * rho_inf * v_inf^2;

    H = CL_delta * q_inf * S;

    % A = [d -d  d -d; 
    %      r  0 -r  0;
    %      0 -r  0  r];
    % 
    % b = [T_x/H; T_y/H; T_z/H];

    % A = [kins.canard.z_cp_13, kins.canard.y_cp_24, kins.canard.z_cp_13, kins.canard.y_cp_24;
    %  kins.canard.x_cp,   -kins.canard.x_cp,    0,  0;
    %  0,  0,   kins.canard.x_cp,  -kins.canard.x_cp];

    C_p = (kins.diameter/2) + (kins.canard.height/2);

    A = [
        C_p/kins.I_x -C_p/kins.I_x C_p/kins.I_x -C_p/kins.I_x;
        kins.x_cp/kins.I_y 0 -kins.x_cp/kins.I_y 0;
        0 -kins.x_cp/kins.I_z 0 kins.x_cp/kins.I_z;
    ];

    % Compute b vector
    b = (1 / (q_inf * kins.canard.S * AeroModel.canard.CL_delta*v_inf)) * ...
        [0; 
         accel_B(2); 
         accel_B(3)];

    cmd = pinv(A) * b;

    canardInput.d1 = cmd(1);
    canardInput.d2 = cmd(2);
    canardInput.d3 = cmd(3);
    canardInput.d4 = cmd(4);

end