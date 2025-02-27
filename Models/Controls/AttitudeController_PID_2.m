function [canardInput, T, err] = AttitudeController_PID_2(x, desiredAttitude, G_1, G_2, dt, kins, inds, AeroModel)

    eulBuff = quat2eul(x(inds.q, :)', 'ZYX');
    yawBuff = eulBuff(:, 1);
    pitchBuff = eulBuff(:, 2);
    rollBuff = eulBuff(:, 3);

    rollCmd = desiredAttitude(3);
    pitchCmd = desiredAttitude(2);
    yawCmd = desiredAttitude(1);

    % Roll Controller
    % err_roll = atan2(sin(rollCmd - rollBuff(1)), cos(rollCmd - rollBuff(1)));
    err_roll = shortest_angle_diff(rollBuff(1), rollCmd);
    err_roll_d = (rollBuff(1) - rollBuff(end)) / ((size(rollBuff,1) - 1) * dt);
    err_roll_int = trapz(rollBuff) * dt;

    T_x = err_roll * G_1(1) + err_roll_int * G_1(2) + err_roll_d * G_1(3);

    % Pitch Controller
    % err_pitch = atan2(sin(pitchCmd - pitchBuff(1)), cos(pitchCmd - pitchBuff(1)));
    err_pitch = shortest_angle_diff(pitchBuff(1), pitchCmd);
    err_pitch_d = (pitchBuff(1) - pitchBuff(end)) / ((size(pitchBuff,1) - 1) * dt);
    err_pitch_int = trapz(pitchBuff) * dt;

    T_y = err_pitch * G_2(1) + err_pitch_int *  G_2(2) + err_pitch_d *  G_2(3);

    % Yaw Controller
    % err_yaw = atan2(sin(yawCmd - yawBuff(1)), cos(yawCmd - yawBuff(1)));
    err_yaw = shortest_angle_diff(yawBuff(1), yawCmd);
    err_yaw_d = (yawBuff(1) - yawBuff(end)) / ((size(yawBuff,1) - 1) * dt);
    err_yaw_int = trapz(yawBuff) * dt;

    T_z = err_yaw * G_2(1) + err_yaw_int *  G_2(2) + err_yaw_d *  G_2(3);

    T = [T_x; T_y; T_z];
    err = [err_yaw; err_pitch; err_roll];

    lla = ecef2lla(x(inds.pos),"wgs84");
    alt = lla(3);
    AtmosphericModel(alt);
    rho_inf = AtmosphericModel.rho_sl;
    v_inf = norm(x(inds.vel, 2));

    q_inf = 0.5 * rho_inf * v_inf^2;

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
    % b = [T_x; T_y; T_z];
    b = (1 / (q_inf * kins.canard.S * AeroModel.canard.CL_delta)) * ...
        [T_x; 
         T_y; 
         T_z];

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

    function err = shortest_angle_diff(current, target)
        err = atan2(sin(target - current), cos(target - current));
    end

end