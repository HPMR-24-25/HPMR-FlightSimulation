function lla = selectLaunchLocation()
    % Create a pop-up figure window
    f = figure('Name', 'Select Launch Location', 'Position', [25 25 1000 700]);

    % Create a geographic axes object within the figure
    geoAxes = geoaxes(f, 'Position', [0.06, 0.1, 0.9, 0.85]);  % Use geographic axes
    geobasemap(geoAxes, 'satellite'); % Set basemap, e.g., 'satellite', 'topographic', 'streets'

    % Enable interative navigation
    zoom(f, 'on');
    pan(f, 'on');
    set(zoom(f), 'Motion', 'both', 'Enable', 'on');

    % Adjustable default view
    % US Region
    % geolimits(geoAxes, [20, 55], [-130, -60]);

    % Allow user to click on the map to select a point
    title(geoAxes, 'Click on the map to select the launch location');
    
    % Wait for the user to click on the map
    try
        [lat, lon] = ginput(1); % Gets longitude and latitude from the clicked point
    catch
        warning('User canceled launch location selection.')
        lla = [NaN, NaN, NaN];
        close(f);
        return;
    end
    
    % Ask for altitude input (optional)
    prompt = {'Enter altitude (m):'};
    dlgtitle = 'Input';
    dims = [1 35];
    definput = {'0'};
    altitude = str2double(inputdlg(prompt, dlgtitle, dims, definput));
    
    % Return the coordinates as LLA (Latitude, Longitude, Altitude)
    lla = [lat, lon, altitude];

    % Display the selected LLA
    disp(['Selected Launch Location (Lat, Lon, Alt): ', num2str(lla)]);
    
    % Close the pop-up figure
    close(f);
end
