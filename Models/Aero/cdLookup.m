function Cd = cdLookup(MachNumber, AoA, AeroModel)
    % Ensure MachNumber and AoA are in the correct format
    if ~isscalar(MachNumber) || ~isscalar(AoA)
        error('MachNumber and AoA must be scalars.');
    end

    % Makima interpolation
    Cd = interp2(AeroModel.MachNumbers, AeroModel.AoA, AeroModel.Cd_table, MachNumber, AoA, 'makima');

    % Check if Cd is NaN, which indicates that the interpolation is out of bounds
    if isnan(Cd)
        warning('Interpolated Cd value is NaN. Check if MachNumber and AoA are within the defined ranges.');
    end
end
