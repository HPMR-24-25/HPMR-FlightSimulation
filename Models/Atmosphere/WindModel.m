function V_wind_ECEF = WindModel(x, inds)

    r_ecef = x(inds.pos);

    lla = ecef2lla(r_ecef', 'WGS84');
    
    lat = lla(1);
    lon = lla(2);
    alt = lla(3);

    V_wind_NED = [2; 2; 0] * (1 + 0.001 * alt);

    R_ET = [
        -sind(lat)*cosd(lon), -sind(lon), -cosd(lat)*cosd(lon);
        -sind(lat)*sind(lon),  cosd(lon), -cosd(lat)*sind(lon);
         cosd(lat),            0,         -sind(lat)
    ];
    
    V_wind_ECEF = R_ET * V_wind_NED;

end