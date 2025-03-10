function lla = selectLaunchLocation()
    % Create figure window
    f = figure('Name', 'Launch Location Selection', 'NumberTitle', 'off', 'MenuBar', 'none');
    
    % Create a geographic axes that fills the figure
    ax = geoaxes('Parent', f, 'Position', [0.1 0.1 0.8 0.8]);
    
    % Set the basemap to satellite imagery
    geobasemap(ax, 'satellite');
    
    % Set initial view to continental US
    % geolimits(ax, [25 50], [-130 -65]);
    
    % Add instructions
    title(ax, 'Click on the map to select launch location');
    
    [lat, lon] = ginput(1);

    prompt = {'Enter altitude (m):'}
    dlgtitle = 'Input';
    dims = [1 35];
    definput = {'0'};
    altitude = str2double(inputdlg(prompt, dlgtitle, dims, definput));

    lla = [lat, lon, altitude];

    disp(['Selected Launch Location (Lat, Lon, Alt): ', num2str(lla)]);

    close(f);
end