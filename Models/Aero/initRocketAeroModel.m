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

    % Normal Force Coefficients

    Cn_Mach = [0.1 0.5 1.1 2.0 5.0, 0.1 0.5 1.1 2.0 5.0, 0.1 0.5 1.1 2.0 5.0, 0.1 0.5 1.1 2.0 5.0]';
    Cn_AoA  = [0 0 0 0 0, 4 4 4 4 4, 8 8 8 8 8, 12 12 12 12 12]';
    Cn_table = [0 0 0 0 0, 2 2 3 2 0.8, 4 4 6.25 3.8 2.5, 6 6 9.25 6 4]';

    CN_lookup = scatteredInterpolant(Cn_Mach, Cn_AoA, Cn_table, 'linear', 'none');

    ModelData.CnLookup = @(inputMach, inputAoA) max(0, CN_lookup(inputMach, inputAoA) * ~isnan(CN_lookup(inputMach, inputAoA)));

    % <<< Guesstimated Values >>>

    % Canard Lift Force Deflection Coefficient
    ModelData.canard.CL_delta = 0.12;

    % Aerodynamic Restoring Moment Coefficients
    ModelData.damping.Cd_x = 0.3;
    ModelData.damping.Cd_y = 0.5;
    ModelData.damping.Cd_z = 0.5;

    % Good estimate is C_n * sideslip angle
    ModelData.Cy_wind = 0.4;
end
