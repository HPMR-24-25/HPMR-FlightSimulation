clear all; close all; clc

% time steup
dt = 1e-3;  % use to increase accuracy
tf = 50;
t = 0:dt:tf-dt;
nt = length(t);

% gravity
g = 32; % ft/s^2
aT = -1.5*g;

% gain
N = 1:0.1:5;

mdist_TPN = zeros(1,length(N));
mdist_FTPN = zeros(1,length(N));
mdist_PPN = zeros(1,length(N));
mdist_FPPN = zeros(1,length(N));

for j = 1:length(N)

    % initial missile conditions
    Vp = 2000;
    HE = -20*pi/180;
    Rpx_i = 0;
    Rpz_i = 10000;

    % initial target conditions
    Vt = 3400;
    B = pi;
    Rtx_i = 40000;
    Rtz_i = 10000;
    Vtx_i = Vt*cos(B);
    Vtz_i = Vt*sin(B);

    % relative positions and velocities
    Rtpx_i = Rtx_i - Rpx_i;
    Rtpz_i = Rtz_i - Rpz_i;

    % line of sight angle
    lambda = atan2(Rtpz_i, Rtpx_i);

    % missile lead angle
    L = asin(Vt*sin(B*lambda)/Vp);

    % missile velocity componenents
    Vpx_i = Vp*cos(lambda+L+HE);
    Vpz_i = Vp*sin(lambda+L+HE);

    % inputs for rk4

    % for missile
    x_0 = [B; Rtx_i; Rtz_i; Rpx_i; Rpz_i; Vtx_i; Vtz_i; Vpx_i; Vpz_i];

    x_TPN = x_0;
    x_FTPN = x_0;
    x_PPN = x_0;
    x_FPPN = x_0;

    xRec_TPN = zeros(length(x_0), nt);
    xRec_TPN(:,1) = x_0;

    xRec_FTPN = zeros(length(x_0), nt);
    xRec_FTPN(:,1) = x_0;

    xRec_PPN = zeros(length(x_0), nt);
    xRec_PPN(:,1) = x_0;

    xRec_FPPN = zeros(length(x_0), nt);
    xRec_FPPN(:,1) = x_0;

    % rk4 Missile
    for i = 1:nt-1

        k1 = dt * TrueProNav(t, x_TPN, N(j), aT);
        k2 = dt * TrueProNav(t, x_TPN + (1/2)*k1, N(j), aT);
        k3 = dt * TrueProNav(t, x_TPN + (1/2)*k2, N(j), aT);
        k4 = dt * TrueProNav(t, x_TPN + k3, N(j), aT);
        x_TPN = x_TPN + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

        xRec_TPN(:, i+1) = x_TPN;

        k1 = dt * FSE_TrueProNav(t, x_FTPN, N(j), aT);
        k2 = dt * FSE_TrueProNav(t, x_FTPN + (1/2)*k1, N(j), aT);
        k3 = dt * FSE_TrueProNav(t, x_FTPN + (1/2)*k2, N(j), aT);
        k4 = dt * FSE_TrueProNav(t, x_FTPN + k3, N(j), aT);
        x_FTPN = x_FTPN + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

        xRec_FTPN(:, i+1) = x_FTPN;

        k1 = dt * PureProNav(t, x_PPN, N(j), aT);
        k2 = dt * PureProNav(t, x_PPN + (1/2)*k1, N(j), aT);
        k3 = dt * PureProNav(t, x_PPN + (1/2)*k2, N(j), aT);
        k4 = dt * PureProNav(t, x_PPN + k3, N(j), aT);
        x_PPN = x_PPN + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

        xRec_PPN(:, i+1) = x_PPN;

        k1 = dt * FSE_PureProNav(t, x_FPPN, N(j), aT);
        k2 = dt * FSE_PureProNav(t, x_FPPN + (1/2)*k1, N(j), aT);
        k3 = dt * FSE_PureProNav(t, x_FPPN + (1/2)*k2, N(j), aT);
        k4 = dt * FSE_PureProNav(t, x_FPPN + k3, N(j), aT);
        x_FPPN = x_FPPN + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

        xRec_FPPN(:, i+1) = x_FPPN;
    end

% relative positions and velocities
Rtpx_TPN = xRec_TPN(2,:) -  xRec_TPN(4,:);
Rtpz_TPN = xRec_TPN(3,:) -  xRec_TPN(5,:);

Rtpx_FTPN = xRec_FTPN(2,:) -  xRec_FTPN(4,:);
Rtpz_FTPN = xRec_FTPN(3,:) -  xRec_FTPN(5,:);

Rtpx_PPN = xRec_PPN(2,:) -  xRec_PPN(4,:);
Rtpz_PPN = xRec_PPN(3,:) -  xRec_PPN(5,:);

Rtpx_FPPN = xRec_FPPN(2,:) -  xRec_FPPN(4,:);
Rtpz_FPPN = xRec_FPPN(3,:) -  xRec_FPPN(5,:);

% range
Rtp_TPN = sqrt(Rtpx_TPN.^2+Rtpz_TPN.^2);

Rtp_FTPN = sqrt(Rtpx_FTPN.^2+Rtpz_FTPN.^2);

Rtp_PPN = sqrt(Rtpx_PPN.^2+Rtpz_PPN.^2);

Rtp_FPPN = sqrt(Rtpx_FPPN.^2+Rtpz_FPPN.^2);

% Time index of tf (time ot intercept)
[mdist_TPN(j),midx_TPN] = min(abs(Rtp_TPN));

[mdist_FTPN(j),midx_FTPN] = min(abs(Rtp_FTPN));

[mdist_PPN(j),midx_PPN] = min(abs(Rtp_PPN));

[mdist_FPPN(j),midx_FPPN] = min(abs(Rtp_FPPN));

end

[min_mdistTPN, min_mdistTPN_idx] = min(mdist_TPN);
opt_N_TPN = N(min_mdistTPN_idx);

[min_mdistFTPN, min_mdistFTPN_idx] = min(mdist_FTPN);
opt_N_FTPN = N(min_mdistFTPN_idx);

[min_mdistPPN, min_mdistPPN_idx] = min(mdist_PPN);
opt_N_PPN = N(min_mdistPPN_idx);

[min_mdistFPPN, min_mdistFPPN_idx] = min(mdist_FPPN);
opt_N_FPPN = N(min_mdistFPPN_idx);


%%
% missile dynamic model
function dx = TrueProNav(t, x, N, aT)

    % Extract State Variables
    B = x(1); % target flight path angle
    Rtx_i = x(2); Rtz_i = x(3); % target inertial position
    Rpx_i = x(4); Rpz_i = x(5); % missile inerital position
    Vtx_i = x(6); Vtz_i = x(7); % target inertial velocity
    Vpx_i = x(8); Vpz_i = x(9); % missile inertial velocity

    % target velocity magnitude
    Vt = sqrt(Vtx_i^2+Vtz_i^2);

    % relative position and velocity
    Rtp_i = [Rtx_i; Rtz_i]-[Rpx_i; Rpz_i];
    Rtpx_i = Rtp_i(1);
    Rtpz_i = Rtp_i(2);
    Vtp_i = [Vtx_i; Vtz_i]-[Vpx_i; Vpz_i];
    Vtpx_i = Vtp_i(1);
    Vtpz_i = Vtp_i(2);

    % Closing Velocity
    Vc = -(Rtpx_i*Vtpx_i+Rtpz_i*Vtpz_i)/norm(Rtp_i);

    % line of sight angle and rate
    lambda = atan2(Rtpz_i,Rtpx_i);
    dlambda = (Rtpx_i*Vtpz_i-Rtpz_i*Vtpx_i)/(norm(Rtp_i)^2);

    % true ProNav
    ap_true = N*Vc*dlambda;

    % missile derivatives
    dRpx_i = Vpx_i;
    dRpz_i = Vpz_i;
    dVpx_i = ap_true*sin(lambda);
    dVpz_i = ap_true*cos(lambda);

    % target derivatives
    dB = aT/Vt;
    dRtx_i = Vt*cos(B);
    dRtz_i = Vt*sin(B);
    dVtx_i = aT * sin(B);
    dVtz_i = aT * cos(B);

    % state derivative
    dx = [dB; dRtx_i; dRtz_i; dRpx_i; dRpz_i; dVtx_i; dVtz_i; dVpx_i; dVpz_i];
end

function dx = FSE_TrueProNav(t, x, N, aT)

    % Extract State Variables
    B = x(1); % target flight path angle
    Rtx_i = x(2); Rtz_i = x(3); % target inertial position
    Rpx_i = x(4); Rpz_i = x(5); % missile inerital position
    Vtx_i = x(6); Vtz_i = x(7); % target inertial velocity
    Vpx_i = x(8); Vpz_i = x(9); % missile inertial velocity

    % target and pursuer velocity magnitude
    Vt = sqrt(Vtx_i^2+Vtz_i^2);
    Vp = sqrt(Vpx_i^2+Vpz_i^2);

    % relative position
    Rtp_i = [Rtx_i; Rtz_i]-[Rpx_i; Rpz_i];
    Rtpx_i = Rtp_i(1);
    Rtpz_i = Rtp_i(2);
    Rtp = sqrt(Rtpx_i^2+Rtpz_i^2);

    % relative velocity
    Vtp_i = [Vtx_i; Vtz_i]-[Vpx_i; Vpz_i];
    Vtpx_i = Vtp_i(1);
    Vtpz_i = Vtp_i(2);

    % Closing Velocity
    Vc = -(Rtpx_i*Vtpx_i+Rtpz_i*Vtpz_i)/norm(Rtp_i);

    % time to go
    t_go = Rtp/Vc;

    % future target position
    fRtx_i = Vtx_i*t_go+Rtx_i;
    fRtz_i = Vtz_i*t_go+Rtz_i;

    % future relative position
    fRtp_i = [fRtx_i; fRtz_i]-[Rpx_i; Rpz_i];
    fRtpx_i = fRtp_i(1);
    fRtpz_i = fRtp_i(2);
    fRtp = sqrt(fRtpx_i^2+fRtpz_i^2);

    % Closing Velocity
    fVc = -(fRtpx_i*Vtpx_i+fRtpz_i*Vtpz_i)/fRtp;

    % line of sight angle and rate
    lambda = atan2(fRtpz_i,fRtpx_i);
    dlambda = (fRtpx_i*Vtpz_i-fRtpz_i*Vtpx_i)/(fRtp^2);

    % true ProNav
    ap_true = N*fVc*dlambda;

    % missile derivatives
    dRpx_i = Vpx_i;
    dRpz_i = Vpz_i;
    dVpx_i = ap_true*sin(lambda);
    dVpz_i = ap_true*cos(lambda);

    % target derivatives
    dB = aT/Vt;
    dRtx_i = Vt*cos(B);
    dRtz_i = Vt*sin(B);
    dVtx_i = aT * sin(B);
    dVtz_i = aT * cos(B);

    % state derivative
    dx = [dB; dRtx_i; dRtz_i; dRpx_i; dRpz_i; dVtx_i; dVtz_i; dVpx_i; dVpz_i];
end

function dx = PureProNav(t, x, N, aT)

    % Extract State Variables
    B = x(1); % target flight path angle
    Rtx_i = x(2); Rtz_i = x(3); % target inertial position
    Rpx_i = x(4); Rpz_i = x(5); % missile inerital position
    Vtx_i = x(6); Vtz_i = x(7); % target inertial velocity
    Vpx_i = x(8); Vpz_i = x(9); % missile inertial velocity

    % target velocity magnitude
    Vt = sqrt(Vtx_i^2+Vtz_i^2);
    Vp = sqrt(Vpx_i^2+Vpz_i^2);

    % relative position and velocity
    Rtp_i = [Rtx_i; Rtz_i]-[Rpx_i; Rpz_i];
    Rtpx_i = Rtp_i(1);
    Rtpz_i = Rtp_i(2);
    Vtp_i = [Vtx_i; Vtz_i]-[Vpx_i; Vpz_i];
    Vtpx_i = Vtp_i(1);
    Vtpz_i = Vtp_i(2);

    % Closing Velocity
    Vc = -(Rtpx_i*Vtpx_i+Rtpz_i*Vtpz_i)/norm(Rtp_i);

    % line of sight angle and rate
    lambda = atan2(Rtpz_i,Rtpx_i);
    dlambda = (Rtpx_i*Vtpz_i-Rtpz_i*Vtpx_i)/(norm(Rtp_i)^2);

    % Heading Error
    HE = atan2(Vpz_i,Vpx_i);

    % true ProNav
    ap_pure = N*Vp*dlambda;

    % missile derivatives
    dRpx_i = Vpx_i;
    dRpz_i = Vpz_i;
    dVpx_i = ap_pure*sin(HE);
    dVpz_i = ap_pure*cos(HE);

    % target derivatives
    dB = aT/Vt;
    dRtx_i = Vt*cos(B);
    dRtz_i = Vt*sin(B);
    dVtx_i = aT * sin(B);
    dVtz_i = aT * cos(B);

    % state derivative
    dx = [dB; dRtx_i; dRtz_i; dRpx_i; dRpz_i; dVtx_i; dVtz_i; dVpx_i; dVpz_i];
end

function dx = FSE_PureProNav(t, x, N, aT)

    % Extract State Variables
    B = x(1); % target flight path angle
    Rtx_i = x(2); Rtz_i = x(3); % target inertial position
    Rpx_i = x(4); Rpz_i = x(5); % missile inerital position
    Vtx_i = x(6); Vtz_i = x(7); % target inertial velocity
    Vpx_i = x(8); Vpz_i = x(9); % missile inertial velocity

    % target and pursuer velocity magnitude
    Vt = sqrt(Vtx_i^2+Vtz_i^2);
    Vp = sqrt(Vpx_i^2+Vpz_i^2);

    % relative position
    Rtp_i = [Rtx_i; Rtz_i]-[Rpx_i; Rpz_i];
    Rtpx_i = Rtp_i(1);
    Rtpz_i = Rtp_i(2);
    Rtp = sqrt(Rtpx_i^2+Rtpz_i^2);

    % relative velocity
    Vtp_i = [Vtx_i; Vtz_i]-[Vpx_i; Vpz_i];
    Vtpx_i = Vtp_i(1);
    Vtpz_i = Vtp_i(2);

    % Closing Velocity
    Vc = -(Rtpx_i*Vtpx_i+Rtpz_i*Vtpz_i)/norm(Rtp_i);

    % time to go
    t_go = Rtp/Vc;

    % future target position
    fRtx_i = Vtx_i*t_go+Rtx_i;
    fRtz_i = Vtz_i*t_go+Rtz_i;

    % future relative position
    fRtp_i = [fRtx_i; fRtz_i]-[Rpx_i; Rpz_i];
    fRtpx_i = fRtp_i(1);
    fRtpz_i = fRtp_i(2);
    fRtp = sqrt(fRtpx_i^2+fRtpz_i^2);

    % Closing Velocity
    fVc = -(fRtpx_i*Vtpx_i+fRtpz_i*Vtpz_i)/fRtp;

    % line of sight angle and rate
    lambda = atan2(fRtpz_i,fRtpx_i);
    dlambda = (fRtpx_i*Vtpz_i-fRtpz_i*Vtpx_i)/(fRtp^2);

    % Heading Error
    HE = atan2(Vpz_i,Vpx_i);

    % true ProNav
    ap_pure = N*Vp*dlambda;

    % missile derivatives
    dRpx_i = Vpx_i;
    dRpz_i = Vpz_i;
    dVpx_i = ap_pure*sin(HE);
    dVpz_i = ap_pure*cos(HE);

    % target derivatives
    dB = aT/Vt;
    dRtx_i = Vt*cos(B);
    dRtz_i = Vt*sin(B);
    dVtx_i = aT * sin(B);
    dVtz_i = aT * cos(B);

    % state derivative
    dx = [dB; dRtx_i; dRtz_i; dRpx_i; dRpz_i; dVtx_i; dVtz_i; dVpx_i; dVpz_i];
end