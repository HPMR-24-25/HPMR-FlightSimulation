function distance = getHaversine(lat_1, lon_1, lat_2, lon_2, const)
    dLat = deg2rad(lat_2 - lat_1);
    dLon = deg2rad(lon_2 - lon_1);

    a = sin(dLat / 2).^2 + cos(lat_1).*cos(lat_2).*(sin(dLon / 2).^2);

    distance = 2*const.R_e*asin(sqrt(a));
end