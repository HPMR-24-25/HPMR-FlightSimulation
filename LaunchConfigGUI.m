function fig = LaunchConfigGUI()
    disp('Creating Launch Configuration GUI...');
    
    % Create the figure
    fig = uifigure('Name', 'Launch Configuration', 'Position', [500, 300, 500, 400]);
    pause(0.5);
    drawnow;

    % Debugging: Confirm GUI creation
    if ~isvalid(fig)
        error('Failed to create Launch Configuration GUI.');
    end
    disp('Launch Configuration GUI successfully created.');

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
                             'HorizontalAlignment', 'left', ...
                             'Tag', 'MotorInfoLabel');  % Add a tag for easier finding

    % --- Checkbox for Thrust Curve Plots ---
    plotCheckBox = uicheckbox(fig, 'Position', [175, 170, 200, 20], ...  % Adjusted position
                              'Text', 'Plot Thrust Curve', 'Value', false);

    % --- Save Button ---
    saveButton = uibutton(fig, 'Text', 'Save Configuration', ...
        'Position', [100, 50, 200, 30], ...
        'ButtonPushedFcn', @(btn, event) saveConfig(locationLabel, motorDropdown, motorInfoLabel, plotCheckBox, fig));
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

    try
        % Call initMotorModel to load the motor data
        motorData = initMotorModel(selectedMotor, false);

        % Store the motorData in the figure for later use
        fig.UserData.motorData = motorData;

        % Display key motor properties
        motorInfoText = sprintf('Isp: %.2f s, Burn Time: %.2f s, Init Mass: %.2f kg', ...
                                motorData.Isp, motorData.t_b, motorData.launchWt);
        
        % Find the motor info label using its tag
        motorInfoLabel = findobj(fig, 'Type', 'uilabel', 'Tag', 'MotorInfoLabel');
        if ~isempty(motorInfoLabel)
            motorInfoLabel.Text = motorInfoText;
        else
            warning('Could not find motor info label in GUI.');
        end
    catch ME
        warning('Error loading motor data: %s', ME.message);
        % Update GUI to show error
        motorInfoLabel = findobj(fig, 'Type', 'uilabel', 'Tag', 'MotorInfoLabel');
        if ~isempty(motorInfoLabel)
            motorInfoLabel.Text = 'Error loading motor data';
        end
    end
end

% --- Function to Select Launch Location ---
function updateLocation(locationLabel)
    disp('Opening Map for Launch Location Selection...');
    drawnow;  % Ensure MATLAB processes UI updates before opening the map

    % Open the selection map immediately
    lla = selectLaunchLocation();

    % Check if location selection was cancelled
    if isempty(lla)
        disp('Launch location selection cancelled. Terminating configuration.');
        if isvalid(locationLabel.Parent)
            delete(locationLabel.Parent);
        end
        error('Configuration cancelled by user.');  % This will stop the simulation
    end

    % Update GUI with selected location
    locationStr = sprintf('Lat: %.4f, Lon: %.4f, Alt: %.1f m', lla(1), lla(2), lla(3));
    locationLabel.Text = locationStr;
end


% --- Function to Save Configuration ---
function saveConfig(locationLabel, motorDropdown, motorInfoLabel, plotCheckBox, fig)
    try
        % Check if location is selected
        if strcmp(locationLabel.Text, 'Not selected')
            uialert(fig, 'Please select a launch location before saving.', 'Location Not Selected');
            return;
        end

        % Check if motor is selected
        if isempty(motorDropdown.Value) || strcmp(motorDropdown.Value, 'No motors found')
            uialert(fig, 'Please select a motor before saving.', 'Motor Not Selected');
            return;
        end

        locationStr = locationLabel.Text;
        lla = extractLLA(locationStr);
        
        % Validate LLA coordinates
        if any(isnan(lla)) || ~all(isfinite(lla))
            uialert(fig, 'Invalid launch location coordinates.', 'Invalid Coordinates');
            return;
        end
        
        motorModel = motorDropdown.Value;    
        motorInfo = motorInfoLabel.Text;
        plotThrust = plotCheckBox.Value;

        % Retrieve stored motor data
        if ~isfield(fig.UserData, 'motorData') || isempty(fig.UserData.motorData)
            uialert(fig, 'Motor data is missing. Please select a motor again.', 'Missing Motor Data');
            return;
        end
        motorData = fig.UserData.motorData;

        % Store everything in a struct
        config = struct('LaunchLocation', locationStr, ...
                        'LLA', lla, ...
                        'MotorModel', motorModel, ...
                        'MotorInfo', motorInfo, ...
                        'PlotThrust', plotThrust, ...
                        'MotorData', motorData);
        
        % save to .mat file
        save('lastConfig.mat', 'config');

        % If thrust plotting is enabled, generate the plots in a new figure
        if plotThrust && ~isempty(motorData)
            plotFig = figure('Name', 'Thrust Curve');
            fplot(motorData.thrustPolar, [0, motorData.t_b], 'r');
            title('Thrust Curve');
            xlabel('Time (s)');
            ylabel('Thrust (N)');
            grid on;
        end

        % Close the configuration GUI
        delete(fig);
        
    catch ME
        % If any error occurs, show it in a dialog and log it
        errordlg(sprintf('Error saving configuration: %s', ME.message), 'Save Error');
        warning('Error in saveConfig: %s', ME.message);
    end
end

% --- Function to Extract LLA ---
function lla = extractLLA(locationStr)
    try
        % Extract numbers from text string "Lat: ..., Lon: ..., Alt: ..."
        nums = sscanf(locationStr, 'Lat: %f, Lon: %f, Alt: %f m');
        if numel(nums) == 3
            lla = nums';
        else
            lla = [NaN, NaN, NaN];
        end
    catch
        lla = [NaN, NaN, NaN];
    end
end
