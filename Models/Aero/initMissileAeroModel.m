function ModelData = initMissileAeroModel()

    MachNumbers = [0.3, 0.5, 0.7, 0.9, 1.2, 1.5, 2.0, 2.5, 3.0];
    AoA = [-10, -5, 0, 5, 10, 15, 20];

    Cd_table = [
        0.35, 0.33, 0.30, 0.32, 0.36, 0.40, 0.42;  % Mach 0.3
        0.40, 0.38, 0.35, 0.37, 0.42, 0.45, 0.48;  % Mach 0.5
        0.45, 0.42, 0.38, 0.40, 0.45, 0.50, 0.55;  % Mach 0.7
        0.50, 0.47, 0.43, 0.45, 0.50, 0.55, 0.60;  % Mach 0.9
        0.55, 0.52, 0.48, 0.50, 0.55, 0.60, 0.65;  % Mach 1.2
        0.60, 0.57, 0.52, 0.55, 0.60, 0.65, 0.70;  % Mach 1.5
        0.65, 0.62, 0.58, 0.60, 0.65, 0.70, 0.75;  % Mach 2.0
        0.70, 0.67, 0.63, 0.65, 0.70, 0.75, 0.80;  % Mach 2.5
        0.75, 0.72, 0.68, 0.70, 0.75, 0.80, 0.85]; % Mach 3.0

    ModelData.MachNumbers = MachNumbers;
    ModelData.AoAs = AoA;
    ModelData.Cd_table = Cd_table;

    % Given data
    % MachNumbers = [0.3, 0.5, 0.7, 0.9, 1.2, 1.5, 2.0, 2.5, 3.0]; % Mach numbers (9 points)
    % AoA = [-10, -5, 0, 5, 10, 15, 20]; % Angles of Attack (7 points)
    % 
    % Cd_table = [
    %     0.35, 0.33, 0.30, 0.32, 0.36, 0.40, 0.42;  % Mach 0.3
    %     0.40, 0.38, 0.35, 0.37, 0.42, 0.45, 0.48;  % Mach 0.5
    %     0.45, 0.42, 0.38, 0.40, 0.45, 0.50, 0.55;  % Mach 0.7
    %     0.50, 0.47, 0.43, 0.45, 0.50, 0.55, 0.60;  % Mach 0.9
    %     0.55, 0.52, 0.48, 0.50, 0.55, 0.60, 0.65;  % Mach 1.2
    %     0.60, 0.57, 0.52, 0.55, 0.60, 0.65, 0.70;  % Mach 1.5
    %     0.65, 0.62, 0.58, 0.60, 0.65, 0.70, 0.75;  % Mach 2.0
    %     0.70, 0.67, 0.63, 0.65, 0.70, 0.75, 0.80;  % Mach 2.5
    %     0.75, 0.72, 0.68, 0.70, 0.75, 0.80, 0.85]; % Mach 3.0
    
    % Create a grid for interpolation
    [M, A] = meshgrid(MachNumbers, AoA);

    ModelData.CdLookup = @(inputMach, inputAoA) interp2(M, A, Cd_table', inputMach, inputAoA, 'makima');
    
    % Example inputs
    % inputMach = 1.0; % desired Mach number
    % inputAoA = 7;   % desired angle of attack (for example)
    % 
    % % Perform interpolation
    % Cd_interp = interp2(M, A, Cd_table', inputMach, inputAoA, 'makima');
end