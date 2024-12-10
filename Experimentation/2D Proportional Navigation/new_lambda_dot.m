clear all; close all; clc

% time steup
dt = 1e-3;  % use to increase accuracy
tf = 12;
t = 0:dt:tf-dt;
nt = length(t);

% gain
N = 3;

% gravity
g = 32; % ft/s^2

% initial missile conditions
aT = -2*g;
Vp = 3000;
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

xRecord = zeros(length(x_0), nt);
xRecord(:,1) = x;

% rk4 Missile
for i = 1:nt-1

%     k1 = MissileDynamicModel(t(i), x,                N, aT);
%     k2 = MissileDynamicModel(t(i)+dt/2, x + dt*k1/2, N, aT);
%     k3 = MissileDynamicModel(t(i)+dt/2, x + dt*k2/2, N, aT);
%     k4 = MissileDynamicModel(t(i)+dt,   x + dt*k3,   N, aT);
%     x = x + dt*(k1 + 2*k2 + 2*k3 + k4)/6;

    k1 = dt * MissileDynamicModel(t, x, N, aT);
    k2 = dt * MissileDynamicModel(t, x + (1/2)*k1, N, aT);
    k3 = dt * MissileDynamicModel(t, x + (1/2)*k2, N, aT);
    k4 = dt * MissileDynamicModel(t, x + k3, N, aT);
    x = x + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

    xRecord(:, i+1) = x;
end

% relative positions and velocities
Rtpx = xRecord(2,:) -  xRecord(4,:);
Rtpz = xRecord(3,:) -  xRecord(5,:);

% range
Rtp = sqrt(Rtpx.^2+Rtpz.^2);

% pursuer velocity
Vp_mag = sqrt(xRecord(8,:).^2 + xRecord(9,:).^2);

% Time index of tf (time ot intercept)
[blah,miss_index] = min(abs(Rtp));


figure(1)
semilogy(t,Rtp,'linewidth', 2)
title('Miss Distance')
xlabel('Time (s)')
ylabel('Miss Distance (ft)')

% plot
figure(2)
plot(xRecord(4,1:miss_index), xRecord(5,1:miss_index),'linewidth', 2);
hold on
plot(xRecord(2,1:miss_index), xRecord(3,1:miss_index),'linewidth', 2);
title('Pursuer and Target')
legend('Pursuer','Target')
xlim([-1000, 41000])
ylim([8000, 13600])
grid on
hold off

% Number of time steps per plot time step%%%%%%%%%%%%%%(why)%%%%%%%
dt_index = 0.1/dt;
 % Engagement full scale
    k = 1;
    for ii = 1:dt_index:(miss_index+dt_index)

        if ii == 1
            figure(3)
            plot(xRecord(4,1), xRecord(5,1), 'ob', 'linewidth', 1, ...
                'markerfacecolor', 'b'); hold on
            plot(xRecord(2,1), xRecord(3,1), 'or', 'linewidth', 1, ...
                'markerfacecolor', 'r');
%             title([PN_type ' ProNav, -20 Deg HE, N = ' num2str(Np)], ...
%                 'fontsize', 16);
%             xlabel('Downrange [ft]', 'fontsize', 16);
%             ylabel('Altitude [ft]', 'fontsize', 16);
%             set(gca, 'fontsize', 16, 'xlim', [0 40000], 'ylim', ...
%                 [6000 12000], 'position', [0.1220 0.1381 0.8388 0.7119]);
%             set(gcf, 'color', 'w', 'position', [10, 80, 1212, 298]);
            grid on
            axis equal

        end

        if ii >= 2

            figure(3)

            plot([xRecord(2,ii), xRecord(2,ii-dt_index)], ...
                [xRecord(3,ii), xRecord(3,ii-dt_index)], ...
                'r-', 'linewidth', 2);

            plot([xRecord(4,ii), xRecord(4,ii-dt_index)], ...
                [xRecord(5,ii), xRecord(5,ii-dt_index)], ...
                'b-', 'linewidth', 2);

        end

        pause(0.1)

%         if strcmp(vid_file,'on')
%             F1(k) = getframe(gcf);
%             k = k+1;
%         end 

    end 

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