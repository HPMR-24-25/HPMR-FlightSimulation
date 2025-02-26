function setupEnv()
%% setupEnv - Configures simulation environment vars

simPaths = {

    %% Models
    'Models'
    fullfile('Models', 'Aero')
    fullfile('Models', 'Motor')
    fullfile('Models', 'Controls')
    fullfile('Models', 'Gravity')
    
    % Atmospheric Model
    fullfile('Models', 'Atmosphere')

    % IMU Model
    fullfile('Models', 'IMU')

    % Kinematics
    fullfile('Models', 'Kinematics')

    %% Controls
    fullfile('Controls')

    %% Guidance
    fullfile('Guidance')

    %% Utilities
    fullfile('Utils');
    
};

simPaths = strjoin(simPaths, ';');
addpath(simPaths);

disp('[Setup] Configured Environment!');

end