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
Vp = 800;
HE = -20*pi/180;
Rpx_i = 0;
Rpz_i = 10000;

% initial target conditions
Vt = 1000;
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

x = x_0;
x_fut = x_0;
x_ZEM = x_0;

xRecord = zeros(length(x_0), nt);
xRecord(:,1) = x_0;

xRecord_fut = zeros(length(x_0), nt);
xRecord_fut(:,1) = x_0;

xRecord_ZEM = zeros(length(x_0), nt);
xRecord_ZEM(:,1) = x_0;

% rk4 Missile
for i = 1:nt-1

    k1 = dt * MissileDynamicModel(t, x, N, aT);
    k2 = dt * MissileDynamicModel(t, x + (1/2)*k1, N, aT);
    k3 = dt * MissileDynamicModel(t, x + (1/2)*k2, N, aT);
    k4 = dt * MissileDynamicModel(t, x + k3, N, aT);
    x = x + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

    xRecord(:, i+1) = x;

    k1 = dt * FutureDynamicModel(t, x_fut, N, aT);
    k2 = dt * FutureDynamicModel(t, x_fut + (1/2)*k1, N, aT);
    k3 = dt * FutureDynamicModel(t, x_fut + (1/2)*k2, N, aT);
    k4 = dt * FutureDynamicModel(t, x_fut + k3, N, aT);
    x_fut = x_fut + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

    xRecord_fut(:, i+1) = x_fut;

    k1 = dt * ZEMDynamicModel(t, x_ZEM, N, aT);
    k2 = dt * ZEMDynamicModel(t, x_ZEM + (1/2)*k1, N, aT);
    k3 = dt * ZEMDynamicModel(t, x_ZEM + (1/2)*k2, N, aT);
    k4 = dt * ZEMDynamicModel(t, x_ZEM + k3, N, aT);
    x_ZEM = x_ZEM + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

    xRecord_ZEM(:, i+1) = x_ZEM;
end

% relative positions and velocities
Rtpx = xRecord(2,:) -  xRecord(4,:);
Rtpz = xRecord(3,:) -  xRecord(5,:);

Rtpx_fut = xRecord_fut(2,:) -  xRecord_fut(4,:);
Rtpz_fut = xRecord_fut(3,:) -  xRecord_fut(5,:);

Rtpx_ZEM = xRecord_ZEM(2,:) -  xRecord_ZEM(4,:);
Rtpz_ZEM = xRecord_ZEM(3,:) -  xRecord_ZEM(5,:);

% range
Rtp = sqrt(Rtpx.^2+Rtpz.^2);

Rtp_fut = sqrt(Rtpx_fut.^2+Rtpz_fut.^2);

Rtp_ZEM = sqrt(Rtpx_ZEM.^2+Rtpz_ZEM.^2);

% pursuer velocity
Vp_mag = sqrt(xRecord(8,:).^2 + xRecord(9,:).^2);

Vp_mag_fut = sqrt(xRecord_fut(8,:).^2 + xRecord_fut(9,:).^2);

Vp_mag_ZEM = sqrt(xRecord_ZEM(8,:).^2 + xRecord_ZEM(9,:).^2);

% Time index of tf (time ot intercept)
[blah,miss_index] = min(abs(Rtp));

[blah_fut,miss_index_fut] = min(abs(Rtp_fut));

[blah_ZEM,miss_index_ZEM] = min(abs(Rtp_ZEM));

% figure(1)
% semilogy(t,Rtp,'linewidth', 2)
% title('Miss Distance')
% xlabel('Time (s)')
% ylabel('Miss Distance (ft)')

% plot
figure(2)
subplot(3,1,1)
plot(xRecord(4,1:miss_index), xRecord(5,1:miss_index),'linewidth', 2);
hold on
plot(xRecord(2,1:miss_index), xRecord(3,1:miss_index),'linewidth', 2);
title('True ProNav', 'Miss Distance = ' + string(blah))
legend('Pursuer','Target')
% xlim([-1000, 41000])
% ylim([7000, 13600])
grid on
hold off
subplot(3,1,2)
plot(xRecord_fut(4,1:miss_index_fut), xRecord_fut(5,1:miss_index_fut),'linewidth', 2);
hold on
plot(xRecord_fut(2,1:miss_index_fut), xRecord_fut(3,1:miss_index_fut),'linewidth', 2);
title('Future True ProNav', 'Miss Distance = ' + string(blah_fut))
legend('Pursuer','Target')
% xlim([-1000, 41000])
% ylim([7000, 13600])
grid on
hold off
subplot(3,1,3)
plot(xRecord_ZEM(4,1:miss_index_ZEM), xRecord_ZEM(5,1:miss_index_ZEM),'linewidth', 2);
hold on
plot(xRecord_ZEM(2,1:miss_index_ZEM), xRecord_ZEM(3,1:miss_index_ZEM),'linewidth', 2);
title('Zero Effort Miss True ProNav', 'Miss Distance = ' + string(blah_ZEM))
legend('Pursuer','Target')
% xlim([-1000, 41000])
% ylim([7000, 13600])
grid on
hold off

% % Number of time steps per plot time step%%%%%%%%%%%%%%(why)%%%%%%%
% dt_index = 0.1/dt;
%  % Engagement full scale
%     k = 1;
%     for ii = 1:dt_index:(miss_index+dt_index)
% 
%         if ii == 1
%             figure(3)
%             plot(xRecord(4,1), xRecord(5,1), 'ob', 'linewidth', 1, ...
%                 'markerfacecolor', 'b'); hold on
%             plot(xRecord(2,1), xRecord(3,1), 'or', 'linewidth', 1, ...
%                 'markerfacecolor', 'r');
% %             title([PN_type ' ProNav, -20 Deg HE, N = ' num2str(Np)], ...
% %                 'fontsize', 16);
% %             xlabel('Downrange [ft]', 'fontsize', 16);
% %             ylabel('Altitude [ft]', 'fontsize', 16);
% %             set(gca, 'fontsize', 16, 'xlim', [0 40000], 'ylim', ...
% %                 [6000 12000], 'position', [0.1220 0.1381 0.8388 0.7119]);
% %             set(gcf, 'color', 'w', 'position', [10, 80, 1212, 298]);
%             grid on
%             axis equal
% 
%         end
% 
%         if ii >= 2
% 
%             figure(3)
% 
%             plot([xRecord(2,ii), xRecord(2,ii-dt_index)], ...
%                 [xRecord(3,ii), xRecord(3,ii-dt_index)], ...
%                 'r-', 'linewidth', 2);
% 
%             plot([xRecord(4,ii), xRecord(4,ii-dt_index)], ...
%                 [xRecord(5,ii), xRecord(5,ii-dt_index)], ...
%                 'b-', 'linewidth', 2);
% 
%         end
% 
%         pause(0.1)
% 
%     end 

%%
% missile dynamic model
function dx = MissileDynamicModel(t, x, N, aT)

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

function dx = FutureDynamicModel(t, x, N, aT)

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

function dx = ZEMDynamicModel(t, x, N, aT)

    % Extract State Variables
    B = x(1); % target flight path angle
    Rtx_i = x(2); Rtz_i = x(3); % target inertial position
    Rpx_i = x(4); Rpz_i = x(5); % missile inerital position
    Vtx_i = x(6); Vtz_i = x(7); % target inertial velocity
    Vpx_i = x(8); Vpz_i = x(9); % missile inertial velocity

    % target and pursuer velocity magnitude
    Vt = sqrt(Vtx_i^2+Vtz_i^2);
    Vp = sqrt(Vpx_i^2+Vpz_i^2);

    % relative position and velocity
    Rtp_i = [Rtx_i; Rtz_i]-[Rpx_i; Rpz_i];
    Rtpx_i = Rtp_i(1);
    Rtpz_i = Rtp_i(2);
    Vtp_i = [Vtx_i; Vtz_i]-[Vpx_i; Vpz_i];
    Vtpx_i = Vtp_i(1);
    Vtpz_i = Vtp_i(2);

    % time to go
    t_go = norm(Rtp_i)/Vp;

    % Closing Velocity
    Vc = -(Rtpx_i*Vtpx_i+Rtpz_i*Vtpz_i)/norm(Rtp_i);

    % line of sight angle and rate
    lambda = atan2(Rtpz_i,Rtpx_i);
    dlambda = (Rtpx_i*Vtpz_i-Rtpz_i*Vtpx_i)/(norm(Rtp_i)^2);

    % zero effort miss inertial frame
    ZEMx_i = Rtpx_i+Vtpx_i*t_go;
    ZEMz_i = Rtpz_i+Vtpz_i*t_go;
    ZEM_i = [ZEMx_i; ZEMz_i];

    % zero effort miss LOS frame
    Cli = [cos(lambda), sin(lambda); -sin(lambda), cos(lambda)];
    ZEM_l = Cli*ZEM_i;
    ZEMx_l = ZEM_l(1);
    ZEMz_l = ZEM_l(2);

    % true ProNav
    ZEMplos = ZEMz_l;
    ap_true = N*ZEMplos/(t_go^2);

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