function [canardInput, T] = AttitudeController_PID(x, desiredAttitude, P, I, D, dt, kins, inds, AeroModel)

    eulBuff = quat2eul(x(inds.q, :)', 'ZYX');
    yawBuff = eulBuff(:, 1);
    pitchBuff = eulBuff(:, 2);
    rollBuff = eulBuff(:, 3);

    rollCmd = desiredAttitude(1);
    pitchCmd = desiredAttitude(2);
    yawCmd = desiredAttitude(3);

    % Roll Controller
    err_roll = rollCmd - rollBuff(2);
    err_roll_d = ((rollCmd - rollBuff(2)) - (rollCmd - rollBuff(1))) / dt;
    err_roll_int = ((rollCmd - rollBuff(2)) - (rollCmd - rollBuff(1))) * dt;

    T_x = err_roll * P + err_roll_int * I + err_roll_d * D;

    % Pitch Controller
    err_pitch = pitchCmd - pitchBuff(2);
    err_pitch_d = ((pitchCmd - pitchBuff(2)) - (pitchCmd - pitchBuff(1))) / dt;
    err_pitch_int = ((pitchCmd - pitchBuff(2)) - (pitchCmd - pitchBuff(1))) * dt;

    T_y = err_pitch * P + err_pitch_int * I + err_pitch_d * D;

    % Yaw Controller
    err_pitch = yawCmd - yawBuff(2);
    err_pitch_d = ((yawCmd - yawBuff(2)) - (yawCmd - yawBuff(1))) / dt;
    err_pitch_int = ((yawCmd - yawBuff(2)) - (yawCmd - yawBuff(1))) * dt;

    T_z = err_pitch * P + err_pitch_int * I + err_pitch_d * D;

    T = [T_x; T_y; T_z];

    lla = ecef2lla(x(inds.pos),"wgs84");
    alt = lla(3);
    AtmosphericModel(alt);

    d = kins.diameter;
    r = kins.x_cp;
    rho_inf = AtmosphericModel.rho_sl;
    v_inf = norm(x(inds.vel, 2));
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
        C_p -C_p C_p -C_p;
        0 kins.x_cp 0 -kins.x_cp;
        -kins.x_cp 0 kins.x_cp 0;
    ];

    % Compute b vector
    b = [T_x; T_y; T_z];
    % b = (1 / (q_inf * kins.canard.S * AeroModel.canard.CL_delta)) * ...
    %     [T_x; 
    %      T_y; 
    %      T_z];

    % cmd = b / A';
    cmd = pinv(A) * b;

    % canardInput.d1 = T_x;
    % canardInput.d2 = T_x;
    % canardInput.d3 = T_x;
    % canardInput.d4 = T_x;

    canardInput.d1 = cmd(1);
    canardInput.d2 = cmd(2);
    canardInput.d3 = cmd(3);
    canardInput.d4 = cmd(4);

end