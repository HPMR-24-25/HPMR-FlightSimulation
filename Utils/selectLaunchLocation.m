function lla = selectLaunchLocation()
    % Create a pop-up figure window
    f = figure('Name', 'Select Launch Location', 'Position', [100 100 600 400]);

    % Create a geographic axes object within the figure
    geoAxes = geoaxes(f);  % Use geographic axes
    geobasemap(geoAxes, 'satellite'); % Set basemap, e.g., 'satellite', 'topographic', 'streets'

    % Allow user to click on the map to select a point
    title(geoAxes, 'Click on the map to select the launch location');
    
    % Wait for the user to click on the map
    [lon, lat] = ginput(1); % Gets longitude and latitude from the clicked point
    
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
