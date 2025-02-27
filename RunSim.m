%% WPI High Power Rocket MQP - Flight Simulator
% Author: Daniel Pearson (djpearson@wpi.edu)
% Version: 12.15.2024

clear variables; close all; clc;

%% Configure constants and model data
const = setupConstants();
% kins = HPMR_MissileKinematics();
kins = HPMR_ModelRocketKinematics();

% Kinematics 
inds = getMissileInds(); % Control State Indices

% Aerodynamics Model
% AeroModel = initMissileAeroModel();
AeroModel = initRocketAeroModel();

% Motor Model
MotorModel = initMotorModel();

ImuModel = getASM330Params();

%% Simulator Config **FOR SIMULINK USE LATER**
% Time Configuration
time.dt = 0.01; % [s] Time Step
time.t0 = 0; % [s] Initial Time
% time.tf = 60*3; % [s] Final Time
time.tf = 200;

simCfg.time = time;

%% Launch Site Initialization
% [launchLat, launchLon, launchAlt] = selectLaunchLocation();
launchLat =  42.2738703; % [deg] Latitude
launchLon = -71.8098593; % [deg] Longitude
launchAlt = 180; % [m] Altitude MSL

launchLLA = [launchLat, launchLon, launchAlt];
currLLA = launchLLA;

launch_ECEF_m = lla2ecef(launchLLA);

accel_ecef = [0; 0; 0];

%% Target Initialization
targetLat = 42.33599546; % [deg] Latitude
targetLon = -71.8098593; % [deg] Longitude
targetAlt = 4752; % [m] Altitude MSL

targetLLA = [targetLat, targetLon, targetAlt];
currTargetLLA = targetLLA;

target_ECEF = lla2ecef(targetLLA);

% Attitude Initialization
yaw_0 = deg2rad(0);
roll_0 = deg2rad(10);
pitch_0 = deg2rad(80);

q_0 = hpmr_eul2quat(yaw_0, pitch_0, roll_0);

eul_0 = hpmr_quat2eul(q_0);

%$ Angular Rate Initialization
w_ib_x = 0.00; % [rad/s]
w_ib_y = 0.00; % [rad/s]
w_ib_z = 0.00; % [rad/s]

% Velocity Initialization
Vx_E_0 = 1e-2; % [m/s]
Vy_E_0 = 1e-2; % [m/s]
Vz_E_0 = 1e-2; % [m/s]

% Initial Mass
m_0 = kins.m_0 + MotorModel.emptyWt + MotorModel.propWt;


%% State Initialization
x_0 = [
    q_0;
    launch_ECEF_m';
    Vx_E_0;
    Vy_E_0;
    Vz_E_0;
    w_ib_x;
    w_ib_y;
    w_ib_z;
    m_0;
];

x_t = x_0;

%% Target State Initialization
% gravity
g = 9.8;
% initial target conditions
Vt = 300;
B = pi;
Rtx_i = target_ECEF(1);
Rty_i = target_ECEF(1);
Rtz_i = target_ECEF(1);
Vtx_i = Vt*cos(B);
Vty_i = Vt*sin(B);
Vtz_i = 0;
aT = 3*g;
x_0_target = [B; Rtx_i; Rty_i; Rtz_i; Vtx_i; Vty_i; Vtz_i];

x_t_target = x_0_target;

%% State Data Storage
t = time.t0;

numTimePts = time.tf / time.dt+1;

tRecord = nan(1, numTimePts);
tRecord(1,1) = t;

xRecord = nan(length(x_0), numTimePts);
xRecord(:,1) = x_t;

accelRecord = nan(length(accel_ecef), numTimePts);
accelRecord(:,1) = accel_ecef;

accelRecordB = nan(length(accel_ecef), numTimePts);
accelRecordB(:,1) = accel_ecef;

xRecord_target = nan(length(x_0_target), numTimePts);
xRecord_target(:,1) = x_t_target;

tSpan = [0, time.tf];  % Start time and end time
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);  % Tolerances for ode45

%% Guidance Storage
cmdHist = zeros(4, numTimePts);
% Initialize buffer and max actuation rate for canards (e.g., 0.1 rad/s)
accelBuffer = nan(size(accel_ecef, 1), 2);
Canard_Buffer = nan(4, 2);
prevCanardInput = struct('d1', 0, 'd2', 0, 'd3', 0, 'd4', 0); % Initial canard deflections
canardInput = struct('d1', 0, 'd2', 0, 'd3', 0, 'd4', 0); % Initial canard deflections
stateBuffer = nan(size(x_t, 1), 20);
attErr = zeros(3, numTimePts);

%% Fill steady state data for set period of time (Simulate Launcher)
steadyStateDuration = 5; % [s]
numSteadyPts = steadyStateDuration / time.dt;

% Pre-Fill steady state values
tRecord(1:numSteadyPts) = linspace(-steadyStateDuration+time.dt, time.t0, numSteadyPts);
xRecord(:, 1:numSteadyPts) = repmat(x_0, 1, numSteadyPts); % Repeat initial state
accelRecord(:, 1:numSteadyPts) = zeros(3, numSteadyPts);
accelRecordB(:, 1:numSteadyPts) = zeros(3, numSteadyPts);
cmdHist(:, 1:numSteadyPts) = zeros(4, numSteadyPts); % Zero canard deflections
colNum = numSteadyPts;
%% Target
for i = 1:numTimePts

    k1 = time.dt * TargetKinematicModel(t, x_t_target, aT);
    k2 = time.dt * TargetKinematicModel(t, x_t_target + (1/2)*k1, aT);
    k3 = time.dt * TargetKinematicModel(t, x_t_target + (1/2)*k2, aT);
    k4 = time.dt * TargetKinematicModel(t, x_t_target + k3, aT);
    x_t_target = x_t_target + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;


    xRecord_target(:, i) = x_t_target;

end

%% Initialize Live Plots
% Setup Video Writer
% videoFile = 'MissileSimulation.mp4';
% v = VideoWriter(videoFile, 'MPEG-4');
% open(v);

% Set up main figure
dataVis = figure('Name', 'Missile Data Visualization');
% COLUMN 1: Orientation Plot
subplot(1,3,1); 
hold on;
ax = gca;
pose = poseplot(ones('quaternion'));
pose.Orientation = quaternion(x_t(inds.q)');

grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Missile Orientation');

% COLUMN 2: Altitude & Velocity
subplot(2,3,2); % Altitude (top half)
altitudePlot = plot(nan, nan, 'r');
title('Altitude Vs. Time');
ylabel('Altitude (m)');
xlabel('Time (s)');
grid on;
hold on;

subplot(2,3,5); % Velocity
velocityPlot = plot(nan, nan, 'r');
title('Velocity Vs. Time');
ylabel('Velocity (m/s)');
xlabel('Time (s)');
grid on;
hold on;

% COLUMN 3: Angular Velocities
subplot(3,3,3); % Angular Velocity X (Roll Rate)
omegaXPlot = plot(nan, nan, 'r');
title('Angular Velocity X (Roll Rate)');
ylabel('Angular Velocity (rad/s)');
xlabel('Time (s)');
grid on;
hold on;

subplot(3,3,6); % Angular Velocity Y (Pitch Rate)
omegaYPlot = plot(nan, nan, 'g');
title('Angular Velocity Y (Pitch Rate)');
ylabel('Angular Velocity (rad/s)');
xlabel('Time (s)');
grid on;
hold on;

subplot(3,3,9); % Angular Velocity Z (Yaw Rate)
omegaZPlot = plot(nan, nan, 'b');
title('Angular Velocity Z (Yaw Rate)');
ylabel('Angular Velocity (rad/s)');
xlabel('Time (s)');
grid on;
hold on;

% Attitude Error Monitor Window
figure('Name', 'Attitude Error Monitoring');

subplot(3,1,1);
rollErrorPlot = plot(nan, nan, 'r');
title('Roll Error Vs. Time');
ylabel('Roll Error (deg)');
xlabel('Time (s)');
grid on;
hold on;

% Guidance Error Plots
subplot(3,1,2);
pitchErrorPlot = plot(nan, nan, 'g');
title('Pitch Error Vs. Time');
ylabel('Pitch Error (deg)');
xlabel('Time (s)');
grid on;
hold on;

subplot(3,1,3);
yawErrorPlot = plot(nan, nan, 'b');
title('Yaw Error Vs. Time');
ylabel('Yaw Error (deg)');
xlabel('Time (s)');
grid on;
hold on;


% Plot update frequency
updateFrequency = 10;
plotCounter = 0;

%% Run Missile Simulation
while(currLLA(3) >= -5)
    colNum = colNum + 1;

    % Update buffer with the latest state; shift older states
    stateBuffer(:, 2:end) = stateBuffer(:, 1:end-1); 
    stateBuffer(:, 1) = x_t;

    % Update Canard buffer with the latest state; shift older states
    Canard_Buffer(:, 2) = Canard_Buffer(:, 1);
    Canard_Buffer(:, 1) = [canardInput.d1; canardInput.d2; canardInput.d3; canardInput.d4];

    %% ECEF to Body
    r_ecef = [x_t(inds.px_ecef); x_t(inds.py_ecef); x_t(inds.pz_ecef)];
    quat = [x_t(inds.qw), x_t(inds.qx), x_t(inds.qy), x_t(inds.qz)];
    lla = ecef2lla(r_ecef', 'WGS84');
    lat = lla(1);
    lon = lla(2);
    alt = lla(3);

    R_ET = [
        -sind(lat)*cosd(lon), -sind(lon), -cosd(lat)*cosd(lon);
        -sind(lat)*sind(lon),  cosd(lon), -cosd(lat)*sind(lon);
        cosd(lat),            0,         -sind(lat)
        ];

    R_TB = quat2rotm(quat);

    R_EB = R_ET' * R_TB;

    % Attempt to control roll between 4s and 18s
    if(t >= 4 && t <= 18)
        accel_cmd_B = [0; 10; 0];
        accel_cmd_ecef = R_EB * accel_cmd_B;

        rollCmd = deg2rad(0);
        pitchCmd = deg2rad(80);
        yawCmd = deg2rad(0);
        eulCmd = [yawCmd; pitchCmd; rollCmd];

        [canardTargetInput, cmdTorque, err] = AttitudeController_PID(stateBuffer, eulCmd, [1.8, 0.1, 0], [1.8, 0.1, 0], time.dt, kins, inds, AeroModel);
        %canardTargetInput = CanardController_PID(x_t, accel_cmd_ecef, Canard_Buffer, 5, 0, 0, time.dt, kins, inds, AeroModel);
        
        % err = [0 0 0];
        attErr(:, colNum) = err;
        
        % canardInput = constrainMissileAcutationLimits(x_t, canardTargetInput, prevCanardInput, kins, time);
        canardInput = canardTargetInput;

        % canardInput.d1 = deg2rad(6);
        % canardInput.d2 = deg2rad(-6);
        % canardInput.d3 = deg2rad(6);
        % canardInput.d4 = deg2rad(-6);

        % Update the historical command for analysis
        cmdHist(:,colNum) = [canardInput.d1; canardInput.d2; canardInput.d3; canardInput.d4];

        % Update previous canard input state for next iteration
        prevCanardInput = canardInput;
    else
        % No roll control, reset canards to 0 rad
        canardInput.d1 = deg2rad(0);
        canardInput.d2 = deg2rad(0);
        canardInput.d3 = deg2rad(0);
        canardInput.d4 = deg2rad(0);
    end

    % Define the anonymous function for ode45 that captures the inputs
    missileModelODE = @(t, x_t) MissileDynamicModel(x_t, t, canardInput, AeroModel, MotorModel, const, kins, inds);

    % Call ode45 for a small time step from current t to t + dt
    [t_out, x_out] = ode45(missileModelODE, [t, t + time.dt], x_t, options);

    % Update time and state variables with the last output from ode45
    t = t_out(end);
    x_t = x_out(end, :)';  % Transpose to maintain consistency with your original state vector format

    % Convert ECEF position to LLA for altitude check
    currLLA = ecef2lla([x_t(inds.px_ecef)', x_t(inds.py_ecef)', x_t(inds.pz_ecef)']);

    %% ECEF to Body
    r_ecef = [x_t(inds.px_ecef); x_t(inds.py_ecef); x_t(inds.pz_ecef)];
    quat = [x_t(inds.qw), x_t(inds.qx), x_t(inds.qy), x_t(inds.qz)];
    lla = ecef2lla(r_ecef', 'WGS84');
    lat = lla(1);
    lon = lla(2);
    alt = lla(3);

    R_ET = [
        -sind(lat)*cosd(lon), -sind(lon), -cosd(lat)*cosd(lon);
        -sind(lat)*sind(lon),  cosd(lon), -cosd(lat)*sind(lon);
         cosd(lat),            0,         -sind(lat)
    ];

    R_TB = quat2rotm(quat);

    R_EB = R_ET' * R_TB;

    % Record the results for future analysis
    xRecord(:, colNum) = x_t;
    tRecord(1, colNum) = t;

    % Extract Acceleration Readings
    [~, accel_ecef] = MissileDynamicModel(x_out(end, :)', t, canardInput, AeroModel, MotorModel, const, kins, inds);

    accelRecord(:, colNum) = accel_ecef;
    accelRecordB(:, colNum) = R_EB'*accel_ecef;

    sensorReading = generateIMU_Readings(x_t, accel_ecef, ImuModel, inds, const);
    
    %% PUT PRONAV HERE
    accel_cmd(:,colNum) = TrueProNav(x_t, xRecord_target(:,colNum), 3, aT);
    
    %% Live Plot Graph Update
    plotCounter = plotCounter + 1;
    if mod(plotCounter, updateFrequency) == 0
        plotCounter = 0; % avoid plotCounter getting too large
        set(pose, 'Orientation', quaternion(x_t(inds.q)'));

        lla = ecef2lla([xRecord(inds.px_ecef, :)', xRecord(inds.py_ecef, :)', xRecord(inds.pz_ecef, :)']);

        set(altitudePlot, 'XData', tRecord, 'YData', lla(:, 3)); % Altitude vs Time
        set(velocityPlot, 'XData', tRecord, 'YData', vecnorm(xRecord(inds.vel, :))); % Velocity vs Time
        set(omegaXPlot, 'XData', tRecord, 'YData', xRecord(inds.w_ib_x,:)); % Angular Vel X
        set(omegaYPlot, 'XData', tRecord, 'YData', xRecord(inds.w_ib_y,:)); % Angular Vel Y
        set(omegaZPlot, 'XData', tRecord, 'YData', xRecord(inds.w_ib_z,:)); % Angular Vel Z

        set(yawErrorPlot, 'XData', tRecord, 'YData', attErr(1, :));
        set(pitchErrorPlot, 'XData', tRecord, 'YData', attErr(2, :));
        set(rollErrorPlot, 'XData', tRecord, 'YData', attErr(3, :));
    
        drawnow;

        % frame = getframe(dataVis);
        % writeVideo(v, frame);
    end

end

% close(v);

% %% Plot Vehicle Trajectory
% lla = ecef2lla([xRecord(inds.px_ecef, :)', xRecord(inds.py_ecef, :)', xRecord(inds.pz_ecef, :)']);
% 
% %% Euler Angles
eulHist = quat2eul(xRecord(inds.q, :)', 'ZYX');

yawHist   = rad2deg(eulHist(:,1));
pitchHist = rad2deg(eulHist(:,2));
rollHist  = rad2deg(eulHist(:,3));

figure('Name', 'Orientation');
plot(tRecord(:), yawHist);
hold on;
plot(tRecord(:), pitchHist);
plot(tRecord(:), rollHist);
hold off;
title("Euler Angles");
legend('Yaw', 'Pitch', 'Roll');

% Atitude Vs Downrange
% downrange = getHaversine(launchLLA(1), launchLLA(2), lla(:,1), lla(:,2), const);
% figure('Name', 'Conops');
% plot(downrange, lla(:,3));
% title("Mission Conops");
% ylabel("Altitude (m)");
% xlabel("Downrange (m)");
% grid on;

% %% Canard Command History
% figure('Name', 'Canard Angles');
% plot(tRecord(:), rad2deg(cmdHist));
% title('Canard Actuation');
% ylabel("Actuation (deg)");
% xlabel("Time (s)");
% grid on;
% legend('Canard 1', 'Canard 2', 'Canard 3', 'Canard 4');

%% Acceleration ECEF
figure('Name', 'Acceleration');
plot(tRecord(:), accelRecord);
title('Acceleration ECEF');
ylabel("Acceleration");
xlabel("Time (s)");
grid on;
legend('x', 'y', 'z');

%% Acceleration Body
figure('Name', 'Acceleration');
plot(tRecord(:), accelRecordB);
title('Acceleration Body');
ylabel("Acceleration");
xlabel("Time (s)");
grid on;
legend('x', 'y', 'z');
xlim([4 20])

% figure('Name', 'Target Position');
% plot3(position_target_ECEF(:,1), position_target_ECEF(:,2), position_target_ECEF(:,3),'linewidth', 2);
% hold on
% plot3(xRecord(inds.px_ecef, :)', xRecord(inds.py_ecef, :)', xRecord(inds.pz_ecef, :)','linewidth', 2);
% title('Target')
% legend('Target', 'Missile')
% grid on
% hold off