function canardInput = AttitudeController_PID(x, desiredAttitude, P, I, D, dt)

    eulBuff = quat2eul(x(1:4, :)', 'ZYX');
    yawBuff = eulBuff(1, :);
    pitchBuff = eulBuff(2, :);
    rollBuff = eulBuff(3, :);

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

end