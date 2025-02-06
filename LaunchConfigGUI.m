function LaunchConfigGUI()
    % Create the figure
    fig = uifigure('Name', 'Launch Configuration', 'Position', [500, 300, 500, 400]);

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
end

% --- Function to Load Available Motor Files ---
function fileNames = getMotorFileNames()
    engineFolder = './Models/Motor/EngineData';
    engineFiles = dir(fullfile(engineFolder, '*.rse'));
    fileNames = {engineFiles.name};
    
    if isempty(fileNames)
        fileNames = {'No motors found'};
    end

    disp('Available Motor Files:');
    disp(fileNames);
end

% --- Function to Load and Display Motor Model ---
function loadMotorModel(dropdown, fig)
    dropdown.Items = getMotorFileNames();
    pause(0.1);

    dropdown.Items = getMotorFileNames();
    selectedMotor = dropdown.Value;
    
    disp(['Selected Motor: ', selectedMotor]);

    if strcmp(selectedMotor, 'No motors found') || isempty(selectedMotor)
        return;
    end
    
    % Call initMotorModel to load the motor data
    motorData = initMotorModel(false);

    % Display key motor properties
    motorInfoText = sprintf('Isp: %.2f s, Burn Time: %.2f s, Init Mass: %.2f kg', ...
                            motorData.Isp, motorData.t_b, motorData.launchWt);
    
    % Find and update the motor info label in the GUI
    motorInfoLabel = findobj(fig, 'Type', 'uilabel', 'Text', 'Motor Info: Select a motor');
    motorInfoLabel.Text = motorInfoText;

    % Store the motorData in the figure for later use
    fig.UserData.motorData = motorData;
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
        motorData = [];
    end

    % Store everything in a struct
    config = struct('LaunchLocation', locationStr, ...
                    'LLA', lla, ...
                    'MotorModel', motorModel, ...
                    'MotorInfo', motorInfo, ...
                    'PlotThrust', plotThrust, ...
                    'MotorData', motorData);
    
    % save to .mat file
    save('lastConfig.mat', 'config');

    % Display in console
    disp('Launch Configuration Saved:');
    disp(config);

    % If thrust plotting is enabled, generate the plots
    if plotThrust && ~isempty(motorData)
        figure('Name', 'Thrust Curve');
        fplot(motorData.thrustPolar, [0, motorData.t_b], 'r');
        title('Thrust Curve');
        xlabel('Time (s)');
        ylabel('Thrust (N)');
        grid on;
    end

    % Close the figure
    close(fig);
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
