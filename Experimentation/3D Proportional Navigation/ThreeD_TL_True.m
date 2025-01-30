clear all; close all; clc

% time steup
dt = 1e-3;  % use to increase accuracy
tf = 50;
t = 0:dt:tf-dt;
nt = length(t);

% gain
N = 3;

% gravity
g = 32; % ft/s^2

% initial missile conditions
Vp = 2000;
HE = -20*pi/180;
Rpx_i = 0;
Rpy_i = 1000; 
Rpz_i = 10000;

% initial target conditions
aT = -3*g;
Vt = 3400;
B = pi;
Rtx_i = 40000;
Rty_i = 0; 
Rtz_i = 10000;

Vtx_i = Vt*cos(B);
Vty_i = 0;
Vtz_i = Vt*sin(B);

% relative positions and velocities
Rtpx_i = Rtx_i - Rpx_i;
Rtpy_i = Rty_i - Rpy_i; 
Rtpz_i = Rtz_i - Rpz_i;
Rtp = sqrt(Rtpx_i^2+Rtpy_i^2+Rtpz_i^2);

% line of sight angle
lambda_xy = atan2(Rtpy_i, Rtpx_i);
lambda_xz = atan2(Rtpz_i, Rtpx_i);
lambda_yz = atan2(Rtpz_i, Rtpy_i);

% missile lead angle
L = asin(Vt*sin(B*lambda_xz)/Vp);

% missile velocity componenents
Vpx_i = Vp*cos(lambda_xz+L+HE);
Vpy_i = 0;
Vpz_i = Vp*sin(lambda_xz+L+HE);

% inputs for rk4
% for missile
x_0 = [B; Rtx_i; Rty_i; Rtz_i; Rpx_i; Rpy_i; Rpz_i; Vtx_i; Vty_i; Vtz_i; Vpx_i; Vpy_i; Vpz_i];

x_TLTrue = x_0;

xRec_TLTrue = zeros(length(x_0), nt);
xRec_TLTrue(:,1) = x_0;

% rk4 Missile
for i = 1:nt-1

    k1 = dt * TL_True(t, x_TLTrue, N, aT);
    k2 = dt * TL_True(t, x_TLTrue + (1/2)*k1, N, aT);
    k3 = dt * TL_True(t, x_TLTrue + (1/2)*k2, N, aT);
    k4 = dt * TL_True(t, x_TLTrue + k3, N, aT);
    x_TLTrue = x_TLTrue + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

    xRec_TLTrue(:, i+1) = x_TLTrue;

end

% relative positions and velocities
Rtpx_TLTrue = xRec_TLTrue(2,:) -  xRec_TLTrue(5,:);
Rtpy_TLTrue = xRec_TLTrue(3,:) -  xRec_TLTrue(6,:);
Rtpz_TLTrue = xRec_TLTrue(4,:) -  xRec_TLTrue(7,:);

% range
Rtp_TLTrue = sqrt(Rtpx_TLTrue.^2+Rtpy_TLTrue.^2+Rtpz_TLTrue.^2);

% pursuer velocity
Vp_mag_TLTrue = sqrt(xRec_TLTrue(11,:).^2 + xRec_TLTrue(12,:).^2 + xRec_TLTrue(13,:).^2);

% Time index of tf (time ot intercept)
[mdist_TLTrue,midx_TLTrue] = min(abs(Rtp_TLTrue));

% plot
figure(1)
plot3(xRec_TLTrue(5,1:midx_TLTrue), xRec_TLTrue(6,1:midx_TLTrue), xRec_TLTrue(7,1:midx_TLTrue),'linewidth', 2);
hold on
plot3(xRec_TLTrue(2,1:midx_TLTrue), xRec_TLTrue(3,1:midx_TLTrue), xRec_TLTrue(4,1:midx_TLTrue),'linewidth', 2);
title('Target Leading True ProNav', 'Miss Distance = ' + string(mdist_TLTrue) + ' ft')
legend('Pursuer','Target')
grid on
hold off

%%
% missile dynamic model
function dx = TL_True(t, x, N, aT)

    % Extract State Variables
    B = x(1); % target flight path angle
    Rtx_i = x(2);  Rty_i = x(3);  Rtz_i = x(4); % target inertial position
    Rpx_i = x(5);  Rpy_i = x(6);  Rpz_i = x(7); % missile inerital position
    Vtx_i = x(8);  Vty_i = x(9);  Vtz_i = x(10); % target inertial velocity
    Vpx_i = x(11); Vpy_i = x(12); Vpz_i = x(13); % missile inertial velocity

    % target and pursuer velocity magnitude
    Vt = sqrt(Vtx_i^2+Vty_i^2+Vtz_i^2);
    Vp = sqrt(Vpx_i^2+Vpy_i^2+Vpz_i^2);

    % relative position
    Rtp_i = [Rtx_i; Rty_i; Rtz_i]-[Rpx_i; Rpy_i; Rpz_i];
    Rtpx_i = Rtp_i(1);
    Rtpy_i = Rtp_i(2);
    Rtpz_i = Rtp_i(3);
    Rtp = sqrt(Rtpx_i^2+Rtpy_i^2+Rtpz_i^2);

    % relative velocity
    Vtp_i = [Vtx_i; Vty_i; Vtz_i]-[Vpx_i; Vpy_i; Vpz_i];
    Vtpx_i = Vtp_i(1);
    Vtpy_i = Vtp_i(2);
    Vtpz_i = Vtp_i(3);
    
    % Target Velocity
    Vtxy = sqrt(Vtx_i^2+Vty_i^2);
    Vtxz = sqrt(Vtx_i^2+Vtz_i^2);
    Vtyz = sqrt(Vty_i^2+Vtz_i^2);

    % Closing Velocity
    Vc_xy = -(Rtpx_i*Vtpx_i+Rtpy_i*Vtpy_i)/sqrt(Rtpx_i^2+Rtpy_i^2);
    Vc_xz = -(Rtpx_i*Vtpx_i+Rtpz_i*Vtpz_i)/sqrt(Rtpx_i^2+Rtpz_i^2);
    Vc_yz = -(Rtpy_i*Vtpy_i+Rtpz_i*Vtpz_i)/sqrt(Rtpy_i^2+Rtpz_i^2);

%     % time to go
%     t_go = Rtp/Vc;
% 
%     % future target position
%     fRtx_i = Vtx_i*t_go+Rtx_i;
%     fRty_i = Vty_i*t_go+Rty_i;
%     fRtz_i = Vtz_i*t_go+Rtz_i;
% 
%     % future relative position
%     fRtp_i = [fRtx_i; fRty_i; fRtz_i]-[Rpx_i; Rpy_i; Rpz_i];
%     fRtpx_i = fRtp_i(1);
%     fRtpy_i = fRtp_i(2);
%     fRtpz_i = fRtp_i(3);
%     fRtp = sqrt(fRtpx_i^2+fRtpy_i^2+fRtpz_i^2);
% 
%     % Closing Velocity
%     fVc = -(fRtpx_i*Vtpx_i+fRtpy_i*Vtpy_i+fRtpz_i*Vtpz_i)/fRtp;
% 
%     % zero effort miss
%     ZEM = Rtp_i+Vtp_i*t_go;
%     e = Rtp_i/Rtp;
%     ZEMn = ZEM - dot(ZEM,e)*e;

    % line of sight angle
    lambda_xy = atan2(Rtpy_i,Rtpx_i);
    lambda_xz = atan2(Rtpz_i,Rtpx_i);
    lambda_yz = atan2(Rtpz_i,Rtpy_i);

    % line of sight angle rate
    dlambda_xy = (Rtpx_i*Vtpy_i - Rtpy_i*Vtpx_i)/sqrt(Rtpx_i^2+Rtpy_i^2);
    dlambda_xz = (Rtpx_i*Vtpz_i - Rtpz_i*Vtpx_i)/sqrt(Rtpx_i^2+Rtpz_i^2);
    dlambda_yz = (Rtpy_i*Vtpz_i - Rtpz_i*Vtpy_i)/sqrt(Rtpy_i^2+Rtpz_i^2);

    % acceleration commands
    nc_xy = N*Vc_xy*dlambda_xy;
    nc_xz = N*Vc_xz*dlambda_xz;
    nc_yz = N*Vc_yz*dlambda_yz;

    % missile derivatives
    dRpx_i = Vpx_i;
    dRpy_i = Vpy_i;
    dRpz_i = Vpz_i;
    dVpx_i = -nc_xy*sin(lambda_xy) - nc_xz*sin(lambda_xz);
    dVpy_i = nc_xy*cos(lambda_xy) - nc_yz*sin(lambda_yz);
    dVpz_i = nc_xz*cos(lambda_xz) + nc_yz*cos(lambda_yz);

    % target derivatives
    dB = aT/Vt;
    dRtx_i = Vt*cos(B);
    dRty_i = 0;
    dRtz_i = Vt*sin(B);
    dVtx_i = aT * sin(B);
    dVty_i = 0;
    dVtz_i = aT * cos(B);

    % state derivative
    dx = [dB; dRtx_i; dRty_i; dRtz_i; dRpx_i; dRpy_i; dRpz_i; dVtx_i; dVty_i; dVtz_i; dVpx_i; dVpy_i; dVpz_i];
end