function ModelData = initRocketAeroModel()

    % Load the aerodynamic data
    aeroData = readtable('RasAero_MissileCoeffs.csv');

    % Extract unique Mach numbers and AoA values
    MachNumbers = aeroData.Mach;
    AoA = aeroData.Alpha;
    Cd_table = aeroData.CD;
    Cl_table = aeroData.CL;

    unique_MachNumbers = unique(MachNumbers);
    unique_AoA = unique(AoA);

    % Store data in the structure
    ModelData.MachNumbers = unique_MachNumbers;
    ModelData.AoAs = unique_AoA;
    ModelData.Cd_table = Cd_table;
    ModelData.Cl_table = Cl_table;

    % Create Scattered Interpolant
    CD_lookup = scatteredInterpolant(MachNumbers, AoA, Cd_table, 'linear', 'none');
    CL_lookup = scatteredInterpolant(MachNumbers, AoA, Cd_table, 'linear', 'none');

    ModelData.CdLookup = @(inputMach, inputAoA) CD_lookup(inputMach, inputAoA);
    ModelData.ClLookup = @(inputMach, inputAoA) CL_lookup(inputMach, inputAoA);

    % <<< Guesstimated Values >>>

    % Canard Lift Force Deflection Coefficient
    ModelData.canard.CL_delta = 5.5;

    % Aerodynamic Restoring Moment Coefficients
    ModelData.damping.Cd_x = 0.3;
    ModelData.damping.Cd_y = 0.5;
    ModelData.damping.Cd_z = 0.5;
end
