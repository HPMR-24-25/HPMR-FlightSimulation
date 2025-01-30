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
aT = -3*g;
Vp = 2000;
HE = -20*pi/180;
Rpx_i = 0;
Rpy_i = 10000;
Rpz_i = 1000;

% initial target conditions
Vt = 3400;
B = pi;
Rtx_i = 40000;
Rty_i = 10000;
Rtz_i = 1000;
Vtx_i = Vt*cos(B);
Vty_i = Vt*sin(B);
Vtz_i = 0;

% relative positions and velocities
Rtpx_i = Rtx_i - Rpx_i;
Rtpy_i = Rty_i - Rpy_i;

% line of sight angle
lambda = atan2(Rtpy_i, Rtpx_i);

% missile lead angle
L = asin(Vt*sin(B*lambda)/Vp);

% missile velocity componenents
Vpx_i = Vp*cos(lambda+L+HE);
Vpy_i = Vp*sin(lambda+L+HE);
Vpz_i = 0;

% inputs for rk4

% for missile
% x_0 = [B; Rtx_i; Rty_i; Rpx_i; Rpy_i; Vtx_i; Vty_i; Vpx_i; Vpy_i];
x_0 = [B; Rtx_i; Rty_i; Rtz_i; Rpx_i; Rpy_i; Rpz_i; Vtx_i; Vty_i; Vtz_i; Vpx_i; Vpy_i; Vpz_i];

x = x_0;

xRec = zeros(length(x_0), nt);
xRec(:,1) = x_0;


% rk4 Missile
for i = 1:nt-1

    k1 = dt * TrueProNav(t, x, N, aT);
    k2 = dt * TrueProNav(t, x + (1/2)*k1, N, aT);
    k3 = dt * TrueProNav(t, x + (1/2)*k2, N, aT);
    k4 = dt * TrueProNav(t, x + k3, N, aT);
    x = x + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

    xRec(:, i+1) = x;

end

% relative positions and velocities
Rtpx = xRec(2,:) -  xRec(5,:);
Rtpy = xRec(3,:) -  xRec(6,:);

% range
Rtp = sqrt(Rtpx.^2+Rtpy.^2);

% pursuer velocity
Vp_mag = sqrt(xRec(8,:).^2 + xRec(9,:).^2);

% Time index of tf (time ot intercept)
[mdist,midx] = min(abs(Rtp));

% plot
figure(1)
% plot3(xRec(4,1:midx), xRec(5,1:midx), 0*(1:midx),'linewidth', 2);
plot3(xRec(5,1:midx), xRec(6,1:midx), xRec(7,1:midx),'linewidth', 2);
hold on
plot3(xRec(2,1:midx), xRec(3,1:midx), xRec(4,1:midx),'linewidth', 2);
title('True ProNav', 'Miss Distance = ' + string(mdist) + ' ft')
legend('Pursuer','Target')
grid on
hold off

%%
% missile dynamic model
% function dx = TrueProNav(t, x, N, aT)
% 
%     % Extract State Variables
%     B = x(1); % target flight path angle
%     Rtx_i = x(2); Rty_i = x(3); % target inertial position
%     Rpx_i = x(4); Rpy_i = x(5); % missile inerital position
%     Vtx_i = x(6); Vty_i = x(7); % target inertial velocity
%     Vpx_i = x(8); Vpy_i = x(9); % missile inertial velocity
% 
%     % target velocity magnitude
%     Vt = sqrt(Vtx_i^2+Vty_i^2);
% 
%     % relative position and velocity
%     Rtp_i = [Rtx_i; Rty_i]-[Rpx_i; Rpy_i];
%     Rtpx_i = Rtp_i(1);
%     Rtpy_i = Rtp_i(2);
%     Vtp_i = [Vtx_i; Vty_i]-[Vpx_i; Vpy_i];
%     Vtpx_i = Vtp_i(1);
%     Vtpy_i = Vtp_i(2);
% 
%     % Closing Velocity
%     Vc = -(Rtpx_i*Vtpx_i+Rtpy_i*Vtpy_i)/norm(Rtp_i);
% 
%     % line of sight angle and rate
%     lambda = atan2(Rtpy_i,Rtpx_i);
%     dlambda = (Rtpx_i*Vtpy_i-Rtpy_i*Vtpx_i)/(norm(Rtp_i)^2);
% 
%     % true ProNav
%     ap_true = N*Vc*dlambda;
% 
%     % missile derivatives
%     dRpx_i = Vpx_i;
%     dRpy_i = Vpy_i;
%     dVpx_i = ap_true*sin(lambda);
%     dVpy_i = ap_true*cos(lambda);
% 
%     % target derivatives
%     dB = aT/Vt;
%     dRtx_i = Vt*cos(B);
%     dRty_i = Vt*sin(B);
%     dVtx_i = aT * sin(B);
%     dVty_i = aT * cos(B);
% 
%     % state derivative
%     dx = [dB; dRtx_i; dRty_i; dRpx_i; dRpy_i; dVtx_i; dVty_i; dVpx_i; dVpy_i];
% end

% missile dynamic model
function dx = TrueProNav(t, x, N, aT)

    % Extract State Variables
    B = x(1); % target flight path angle
    Rtx_i = x(2);  Rty_i = x(3);  Rtz_i = x(4); % target inertial position
    Rpx_i = x(5);  Rpy_i = x(6);  Rpz_i = x(7); % missile inerital position
    Vtx_i = x(8);  Vty_i = x(9);  Vtz_i = x(10); % target inertial velocity
    Vpx_i = x(11); Vpy_i = x(12); Vpz_i = x(13); % missile inertial velocity

    % target velocity magnitude
    Vt = sqrt(Vtx_i^2+Vty_i^2);

    % relative position and velocity
    Rtp_i = [Rtx_i; Rty_i]-[Rpx_i; Rpy_i];
    Rtpx_i = Rtp_i(1);
    Rtpy_i = Rtp_i(2);
    Vtp_i = [Vtx_i; Vty_i]-[Vpx_i; Vpy_i];
    Vtpx_i = Vtp_i(1);
    Vtpy_i = Vtp_i(2);

    % Closing Velocity
    Vc = -(Rtpx_i*Vtpx_i+Rtpy_i*Vtpy_i)/norm(Rtp_i);

    % line of sight angle and rate
    lambda = atan2(Rtpy_i,Rtpx_i);
    dlambda = (Rtpx_i*Vtpy_i-Rtpy_i*Vtpx_i)/(norm(Rtp_i)^2);

    % true ProNav
    ap_true = N*Vc*dlambda;

    % missile derivatives
    dRpx_i = Vpx_i;
    dRpy_i = Vpy_i;
    dRpz_i = 0;
    dVpx_i = ap_true*sin(lambda);
    dVpy_i = ap_true*cos(lambda);
    dVpz_i = 0;

    % target derivatives
    dB = aT/Vt;
    dRtx_i = Vt*cos(B);
    dRty_i = Vt*sin(B);
    dRtz_i = 0;
    dVtx_i = aT * sin(B);
    dVty_i = aT * cos(B);
    dVtz_i = 0;

    % state derivative
    dx = [dB; dRtx_i; dRty_i; dRtz_i; dRpx_i; dRpy_i; dRpz_i; dVtx_i; dVty_i; dVtz_i; dVpx_i; dVpy_i; dVpz_i];
end