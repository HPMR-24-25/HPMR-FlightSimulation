function ModelData = initMotorModel(motorPlots)
%% Function: initMotorModel
% Author: Daniel Pearson (djpearson@wpi.edu)
% Version: 9.16.2024
% Inputs:
%   plotThrustCurve - (T/F)
% Outputs:
%   ModelData - Struct containing motor constants and thrust curve

if ~exist('motorPlots', 'var')
    motorPlots = false;
end

% List all .rse files in the 'Engine Data' directory
engineFolder = './Models/Motor/EngineData';
engineFiles = dir(fullfile(engineFolder, '*.rse'));

% Check if there are .rse files in the directory
if isempty(engineFiles)
    error('No .rse files found in the Engine Data folder.');
end

% Create a list of file names for user selection
fileNames = {engineFiles.name};

% Display a selection dialog to the user
[selectedIndex, ok] = listdlg('PromptString', 'Select a motor file:', ...
                              'SelectionMode', 'single', ...
                              'ListString', fileNames);

% If the user cancels, exit the function
if ~ok
    disp('No file selected. Exiting...');
    return;
end

% Get the selected file name
selectedFile = fileNames{selectedIndex};

% Load the selected .rse file
motorDoc = xmlread(fullfile(engineFolder, selectedFile));

engDataNodes = motorDoc.getElementsByTagName('eng-data');

% Initialize arrays to store data
time = [];
thrust = [];
mass = [];
cg = [];

engConstsNode = motorDoc.getElementsByTagName('engine').item(0);

ModelData.Isp = str2double(engConstsNode.getAttribute('Isp'));
ModelData.t_b = str2double(engConstsNode.getAttribute('burn-time'));

% Loop through each <eng-data> element and extract attributes
for i = 0:engDataNodes.getLength-1
    % Get the current <eng-data> node
    engDataNode = engDataNodes.item(i);
    
    % Extract attributes: t (time), f (thrust), m (mass), cg (center of gravity)
    timeVal = str2double(engDataNode.getAttribute('t'));
    thrustVal = str2double(engDataNode.getAttribute('f'));
    massVal = str2double(engDataNode.getAttribute('m'));
    cgVal = str2double(engDataNode.getAttribute('cg'));
    % ispVal = str2double(engDataNode.)
    
    % Append values to the arrays
    time(end+1) = timeVal;
    thrust(end+1) = thrustVal;
    mass(end+1) = massVal/1000; % [g] to [kg]
    cg(end+1) = cgVal;
end

%% Thrust Curve

% Makima interpolate into anonymous function
% thrustPolar = @(t) interp1(time, thrust, t, 'makima');
% 
% %% Prop Mass Curve
% 
% massPolar = @(t) interp1(time, mass, t, 'spline');

%% Thrust Curve
% Interpolate thrust data, return 0 if out of time bounds
thrustPolar = @(t) max(0, ...
                    (t >= min(time) & t <= max(time)) ...
                    .* interp1(time, thrust, t, 'linear') ...
                    + (t < min(time) | t > max(time)) * 0);

%% Mass Curve
% Interpolate prop mass data, return 0 if out of time bounds
massPolar = @(t) (t >= min(time) & t <= max(time)) ...
                    .* interp1(time, mass, t, 'linear') ...
                    + (t < min(time) | t > max(time)) * 0;


%% Mass Flow Rate
% Define a small step size for the numerical derivative
dt = 1e-6;  % Small time step for finite difference

% Define m_dotPolar using a central difference method for the derivative
m_dotPolar = @(t) max(0, ...
                    (t >= min(time) & t <= max(time)) ...
                    .* (-(massPolar(t + dt) - massPolar(t - dt)) / (2 * dt)) ...
                    + (t < min(time) | t > max(time)) * 0);

ModelData.thrustPolar = thrustPolar;
ModelData.massPolar   = massPolar;
ModelData.m_dotPolar  = m_dotPolar;

if(motorPlots)
    figure('Name', 'Thrust Curve');
    scatter(time, thrust, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
    hold on;
    fplot(thrustPolar, [min(time), max(time)], 'r');
    hold off;
    title('Thrust Curve');
    xlabel('Thrust (N)');
    ylabel('Time (s)');
    legend('ThrustPTS', 'Function');
    grid on;

    figure('Name', 'Mass Curve');
    scatter(time, mass, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
    hold on;
    fplot(massPolar, [min(time), max(time)], 'r');
    hold off;
    title('Mass Curve');
    ylabel('Mass (kg)');
    xlabel('Time (s)');
    legend('MassPTS', 'Function');
    grid on;

    figure('Name', 'MDot Curve');
    fplot(m_dotPolar, [min(time), max(time)], 'r');
    title('Mass Flowrate Curve');
    ylabel('M_dot (kg/s)');
    xlabel('Time (s)');
    grid on;
end

end
