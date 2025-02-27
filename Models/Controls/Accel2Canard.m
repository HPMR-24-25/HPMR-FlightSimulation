function canardInput = Accel2Canard(x, accel_cmd_ecef, kins, inds, AeroModel)
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
    
    rho_inf = AtmosphericModel.rho_sl;
    v_inf = norm(x(inds.vel));

    q_inf = 0.5 * rho_inf * v_inf^2;

    C_p = (kins.diameter/2) + (kins.canard.height/2);

    K = 1; %some kind of gain

    A = [1 -1 1 -1;
         1 0 -1 0;
         0 -1 0 1];

    b = [0*kins.I_x/(C_p*q_inf * kins.canard.S * AeroModel.canard.CL_delta*v_inf)      *(K*accel_B(1)+v_inf*(kins.I_z-kins.I_y)*w(2)*w(3)/kins.I_x);
         kins.I_y/(kins.x_cp*q_inf * kins.canard.S * AeroModel.canard.CL_delta*v_inf)*(K*accel_B(2)+v_inf*(kins.I_x-kins.I_z)*w(1)*w(3)/kins.I_y);
         kins.I_z/(kins.x_cp*q_inf * kins.canard.S * AeroModel.canard.CL_delta*v_inf)*(K*accel_B(3)+v_inf*(kins.I_y-kins.I_x)*w(2)*w(1)/kins.I_z)];

    cmd = pinv(A) * b;

    canardInput.d1 = cmd(1);
    canardInput.d2 = cmd(2);
    canardInput.d3 = cmd(3);
    canardInput.d4 = cmd(4);
end