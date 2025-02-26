function dx = TargetKinematicModel(t, x, aT)

    % Extract State Variables
    B = x(1); % target flight path angle
    Rtx_i = x(2);  Rty_i = x(3);  Rtz_i = x(4); % target inertial position
    Vtx_i = x(5);  Vty_i = x(6);  Vtz_i = x(7); % target inertial velocity

    % target velocity magnitude
    Vt = sqrt(Vtx_i^2+Vty_i^2+Vtz_i^2);

    % target derivatives
    dB = aT/Vt;
    dRtx_i = Vt*cos(B);
    dRty_i = Vt*sin(B);
    dRtz_i = 0;
    dVtx_i = aT * sin(B);
    dVty_i = aT * cos(B);
    dVtz_i = 0;

    % state derivative
    dx = [dB; dRtx_i; dRty_i; dRtz_i; dVtx_i; dVty_i; dVtz_i];
end