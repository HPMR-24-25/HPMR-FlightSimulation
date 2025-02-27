function fig = LaunchConfigGUI()
    % Create the figure
    fig = uifigure('Name', 'Launch Configuration', 'Position', [500, 300, 500, 400]);

    pause(0.5);
    drawnow;

    % --- Select Launch Location Button ---
    uilabel(fig, 'Position', [20, 340, 120, 20], 'Text', 'Launch Location:');
    locationLabel = uilabel(fig, 'Position', [175, 340, 250, 20], 'Text', 'Not selected');
    selectLocButton = uibutton(fig, 'Text', 'Select Location', ...
        'Position', [175, 310, 120, 30], ...
        'ButtonPushedFcn', @(btn, event) updateLocation(locationLabel));

    % --- Motor Selection Dropdown ---
    uilabel(fig, 'Position', [20, 250, 100, 20], 'Text', 'Motor Model:');
    motorDropdown = uidropdown(fig, 'Position', [175, 255, 200, 25], ...
        'Items', {}, ...
        'Value', '', ...
        'Enable', 'on', ...
        'Editable', 'on', ...
        'ValueChangedFcn', @(dd, event) loadMotorModel(dd, fig));
    pause(0.5);
    motorDropdown.Items = getMotorFileNames();

    % --- Motor Info Display ---
    motorInfoLabel = uilabel(fig, 'Position', [20, 200, 400, 30], ...
                             'Text', 'Motor Info: Select a motor', ...
                             'HorizontalAlignment', 'left');

    % --- Checkbox for Thrust Curve Plots ---
    plotCheckBox = uicheckbox(fig, 'Position', [175, 200, 200, 20], ...
                              'Text', 'Plot Thrust Curve', 'Value', false);

    % --- Save Button ---
    saveButton = uibutton(fig, 'Text', 'Save Configuration', ...
        'Position', [100, 50, 200, 30], ...
        'ButtonPushedFcn', @(btn, event) saveConfig(locationLabel, motorDropdown, motorInfoLabel, plotCheckBox, fig));

    % Debugging: Confirm GUI was created
    if isvalid(fig)
        disp('Launch Configuration GUI successfully created.');
    else
        error('Failed to create Launch Configuration GUI.');
    end
end

% --- Function to Load Available Motor Files ---
function fileNames = getMotorFileNames()
    engineFolder = './Models/Motor/EngineData';
    
    % Ensure directory exists
    if ~isfolder(engineFolder)
        warning('EngineData folder does not exist: %s', engineFolder);
        fileNames = {'No motors found'};
        return;
    end

    engineFiles = dir(fullfile(engineFolder, '*.rse'));
    fileNames = {engineFiles.name};
    
    if isempty(fileNames)
        fileNames = {'No motors found'};
    end
end

% --- Function to Load and Display Motor Model ---
function loadMotorModel(dropdown, fig)
    % Get selected motor file from dropdown
    selectedMotor = dropdown.Value;

    if strcmp(selectedMotor, 'No motors found') || isempty(selectedMotor)
        warning('No motor selected in dropdown.')
        return;
    end

    % Call initMotorModel to load the motor data
    motorData = initMotorModel(selectedMotor, false);

    % Store the motorData in the figure for later use
    fig.UserData.motorData = motorData;

    % Display key motor properties
    motorInfoText = sprintf('Isp: %.2f s, Burn Time: %.2f s, Init Mass: %.2f kg', ...
                            motorData.Isp, motorData.t_b, motorData.launchWt);
    
    % Find and update the motor info label in the GUI
    motorInfoLabel = findobj(fig, 'Type', 'uilabel', 'Text', 'Motor Info: Select a motor');
    motorInfoLabel.Text = motorInfoText;
end

% --- Function to Select Launch Location ---
function updateLocation(locationLabel)
    lla = selectLaunchLocation();
    locationStr = sprintf('Lat: %.4f, Lon: %.4f, Alt: %.1f m', lla(1), lla(2), lla(3));
    locationLabel.Text = locationStr;
end

% --- Function to Save Configuration ---
function saveConfig(locationLabel, motorDropdown, motorInfoLabel, plotCheckBox, fig)
    locationStr = locationLabel.Text;
    lla = extractLLA(locationStr);
    motorModel = motorDropdown.Value;    
    motorInfo = motorInfoLabel.Text;
    plotThrust = plotCheckBox.Value;

    % Retrieve stored motor data
    if isfield(fig.UserData, 'motorData')
        motorData = fig.UserData.motorData;
    else
        warning('Motor model data is missing in GUI. Please select a motor.');
        motorData = [];
    end

    % Debugging output
    disp('Saving Configuration...');
    disp(['Launch Location: ', locationStr]);
    disp(['Selected Motor: ', motorModel]);
    disp('Motor Data:');
    disp(motorData);  % Should not be empty

    % Store everything in a struct
    config = struct('LaunchLocation', locationStr, ...
                    'LLA', lla, ...
                    'MotorModel', motorModel, ...
                    'MotorInfo', motorInfo, ...
                    'PlotThrust', plotThrust, ...
                    'MotorData', motorData);
    
    % save to .mat file
    save('lastConfig.mat', 'config');

    % If thrust plotting is enabled, generate the plots
    if plotThrust && ~isempty(motorData)
        figure('Name', 'Thrust Curve');
        fplot(motorData.thrustPolar, [0, motorData.t_b], 'r');
        title('Thrust Curve');
        xlabel('Time (s)');
        ylabel('Thrust (N)');
        grid on;
    end

    disp('Saving configuration...');

    if isvalid(fig)
        pause(0.5);
        delete(fig);
        disp('Configuration GUI was forced closed.');
    else
        disp('Configuration GUI was already closed.');
    end
end

function lla = extractLLA(locationStr)
    % Extract numbers from text string "Lat: ..., Lon: ..., Alt: ..."
    nums = sscanf(locationStr, 'Lat: %f, Lon: %f, Alt: %f m');
    if numel(nums) == 3
        lla = nums';
    else
        lla = [0, 0, 0];
    end
end
