function accel_cmd = TrueProNav(x_m, x_t, N, aT)

    % Extract State Variables from Missile
    Rpx_i = x_m(5);  Rpy_i = x_m(6);  Rpz_i = x_m(7); % missile inerital position
    Vpx_i = x_m(8); Vpy_i = x_m(9); Vpz_i = x_m(10); % missile inertial velocity

    % Extract State Variables from Target
    B = x_t(1); % target flight path angle
    Rtx_i = x_t(2);  Rty_i = x_t(3);  Rtz_i = x_t(4); % target inertial position
    Vtx_i = x_t(5);  Vty_i = x_t(6);  Vtz_i = x_t(7); % target inertial velocity

    % target velocity magnitude
    Vt = sqrt(Vtx_i^2+Vty_i^2+Vtz_i^2);

    % relative position and velocity
    Rtp_i = [Rtx_i; Rty_i; Rtz_i]-[Rpx_i; Rpy_i; Rpz_i];
    Vtp_i = [Vtx_i; Vty_i; Vtz_i]-[Vpx_i; Vpy_i; Vpz_i];

    % LOS Unit Vector
    R_vec = Rtp_i/norm(Rtp_i);

    % Closing Velocity
    Vc = -dot(Vtp_i, R_vec);

    % time to go
    t_go = norm(Rtp_i)/Vc;

    % future target position
    fRtx_i = Vtx_i*t_go+Rtx_i;
    fRty_i = Vty_i*t_go+Rty_i;
    fRtz_i = Vtz_i*t_go+Rtz_i;

    % future relative position
    fRtp_i = [fRtx_i; fRty_i; fRtz_i]-[Rpx_i; Rpy_i; Rpz_i];

    % Future LOS Unit Vector
    fR_vec = fRtp_i/norm(fRtp_i);

    % Future Closing Velocity
    fVc = -dot(Vtp_i, fR_vec);

    % Future time to go
    ft_go = norm(fRtp_i)/fVc;

    % Zero Effort Miss
    ZEM = fRtp_i + Vtp_i*ft_go;
    ZEMn = ZEM - dot(ZEM,fR_vec)*fR_vec;

    % missile derivatives
    dRpx_i = Vpx_i;
    dRpy_i = Vpy_i;
    dRpz_i = Vpz_i;
    dVpx_i = N*ZEMn(1)/(ft_go^2);
    dVpy_i = N*ZEMn(2)/(ft_go^2);
    dVpz_i = N*ZEMn(3)/(ft_go^2);

    % target derivatives
    dB = aT/Vt;
    dRtx_i = Vt*cos(B);
    dRty_i = Vt*sin(B);
    dRtz_i = 0;
    dVtx_i = aT * sin(B);
    dVty_i = aT * cos(B);
    dVtz_i = 0;

    % state derivative
    accel_cmd = [dVpx_i; dVpy_i; dVpz_i];
end