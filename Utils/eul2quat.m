% function q_total = eul2quat(roll, pitch, yaw)
%     % Function to convert Roll, Pitch, Yaw angles to a quaternion
%     % Inputs: roll, pitch, yaw (in radians)
% 
%     % Half-angles for quaternion computation
%     half_roll = roll / 2;
%     half_pitch = pitch / 2;
%     half_yaw = yaw / 2;
% 
%     % Quaternion for roll (about X axis)
%     q_roll = [cos(half_roll), sin(half_roll), 0, 0]; % [qw, qx, qy, qz]
% 
%     % Quaternion for pitch (about Y axis)
%     q_pitch = [cos(half_pitch), 0, sin(half_pitch), 0]; % [qw, qx, qy, qz]
% 
%     % Quaternion for yaw (about Z axis)
%     q_yaw = [cos(half_yaw), 0, 0, sin(half_yaw)]; % [qw, qx, qy, qz]
% 
%     % Quaternion multiplication: q_total = q_yaw * q_pitch * q_roll
%     q_total = quatmultiply(quatmultiply(q_yaw, q_pitch), q_roll);
% end