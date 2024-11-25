function canardInput = RollPitchYawController_PID(x, rollCmd, pitchCmd, yawCmd, P_roll, I_roll, D_roll, P_pitch, I_pitch, D_pitch, P_yaw, I_yaw, D_yaw, dt)
% RollPitchYawController_PID - Simple PID controller for roll, pitch, and yaw
% Controls roll, pitch, and yaw with PID control.
% Inputs:
%   x - Current state [quaternion, other state variables if needed]
%   rollCmd - [rad] Commanded Roll
%   pitchCmd - [rad] Commanded Pitch
%   yawCmd - [rad] Commanded Yaw
%   P_roll, I_roll, D_roll - PID gains for roll
%   P_pitch, I_pitch, D_pitch - PID gains for pitch
%   P_yaw, I_yaw, D_yaw - PID gains for yaw
%   dt - Time step for derivative and integral calculations
% Outputs:
%   canardInput - Struct containing actuation commands for each canard

    % Extract Euler angles (in ZYX order) from quaternion
    eulBuff = quat2eul(x(1:4, :)', 'ZYX')';
    yawBuff = eulBuff(1,:);
    pitchBuff = eulBuff(2,:);
    rollBuff = eulBuff(3,:);

    % Roll control
    err_roll = rollCmd - rollBuff(2);
    err_roll_d = ((rollCmd - rollBuff(2)) - (rollCmd - rollBuff(1))) / dt;
    err_roll_int = ((rollCmd - rollBuff(2)) - (rollCmd - rollBuff(1))) * dt;
    cmd_roll = err_roll * P_roll + err_roll_int * I_roll + err_roll_d * D_roll;

    % Pitch control
    err_pitch = pitchCmd - pitchBuff(2);
    err_pitch_d = ((pitchCmd - pitchBuff(2)) - (pitchCmd - pitchBuff(1))) / dt;
    err_pitch_int = ((pitchCmd - pitchBuff(2)) - (pitchCmd - pitchBuff(1))) * dt;
    cmd_pitch = err_pitch * P_pitch + err_pitch_int * I_pitch + err_pitch_d * D_pitch;

    % Yaw control
    err_yaw = yawCmd - yawBuff(2);
    err_yaw_d = ((yawCmd - yawBuff(2)) - (yawCmd - yawBuff(1))) / dt;
    err_yaw_int = ((yawCmd - yawBuff(2)) - (yawCmd - yawBuff(1))) * dt;
    cmd_yaw = err_yaw * P_yaw + err_yaw_int * I_yaw + err_yaw_d * D_yaw;

    % Combine pitch, yaw, and roll control commands for each canard
    % canardInput.d1 = cmd_roll + cmd_pitch + cmd_yaw;
    % canardInput.d2 = cmd_roll - cmd_pitch - cmd_yaw;
    % canardInput.d3 = cmd_roll + cmd_pitch - cmd_yaw;
    % canardInput.d4 = cmd_roll - cmd_pitch + cmd_yaw;
    canardInput.d1 = cmd_roll;
    canardInput.d2 = cmd_roll;
    canardInput.d3 = cmd_roll;
    canardInput.d4 = cmd_roll;
end
