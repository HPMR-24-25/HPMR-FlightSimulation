function q = hpmr_eul2quat(roll, pitch, yaw)
% XYZ SEQUENCE

q = [cos(yaw/2); 0; 0; sin(yaw/2)] * [cos(pitch/2); 0; sin(pitch/2); 0] * [cos(roll/2); sin(roll/2); 0; 0];

end