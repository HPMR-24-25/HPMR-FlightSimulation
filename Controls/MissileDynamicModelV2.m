function x_dot = MissileDynamicModel(x, t, AeroModel, MotorModel, const, kins, inds)

    r_ecef = [x(inds.px_ecef); x(inds.py_ecef); x(inds.pz_ecef)];

    v_ecef = [x(inds.vx_ecef); x(inds.vy_ecef); y(inds.vz_ecef)];

    quat = [x(inds.qw), x(inds.qx), x(inds.qy), x(inds.qz)];

    lla = ecef2lla(r_ecef', 'WGS84');

    lat = lla(1);
    lon = lla(2);
    alt = lla(3);

    atmo = AtmosphericModel(alt);

    rho_alt = atmo.getDensity();

    a = sqrt(const.gamma_Air*const.R_air*atmo.getTemperature());

    M = norm(v_ecef) / a;

    % Moment of Inertia
    I_x = 26.413;
    I_y = 26.413;
    I_z = 0.107;

    %% Rotation Matrix Setup

    R_ET = [
        -sind(lat)*cosd(lon), -sind(lon), -cosd(lat)*cosd(lon);
        -sind(lat)*sind(lon),  cosd(lon), -cosd(lat)*sind(lon);
         cosd(lat),            0,         -sind(lat)
    ];

    R_TB = quat2rotm(quat);

    R_EB = R_ET * R_TB;

    v_hat_E = v_ecef / norm(v_ecef);

    v_hat_B = R_EB' * v_hat_E;

    D_B = [
        0.5*rho_alt*cdPolar(M)*kins.S*norm(v_ecef)^2 * v_hat_B(1);
        0.5*rho_alt*cdPolar(M)*kins.S*norm(v_ecef)^2 * v_hat_B(2);
        0.5*rho_alt*cdPolar(M)*kins.S*norm(v_ecef)^2 * v_hat_B(3);
    ];

    T_motor = MotorModel.thrustPolar(t);

    T_B = [
        T_motor * v_hat_B(1);
        T_motor * v_hat_B(2);
        T_motor * v_hat_B(3);
    ];

    m_dot = MotorModel.m_dotPolar(t);

    D_E = R_EB * D_B;
    T_E = R_EB * T_B;

    vx_dot = (-D_E  + T_E / x(inds.mass)) + ;



end