function ypr = dcm_to_ypr(Q)
yaw = atan2d_0_360(Q(1,2), Q(1,1));
pitch = asind(-Q(1,3));
roll = atan2d_0_360(Q(2,3),Q(3,3));
ypr = [yaw, pitch, roll];
end