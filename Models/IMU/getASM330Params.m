function consts = getASM330Params()

    %% Gyroscope Parameters

    % Angular Random Walk
    consts.gyro.arw.xy = 0.029922; % [deg/sqrt(hz)] 
    consts.gyro.arw.z  = 0.015158; % [deg/sqrt(hz)]

    % Bias Instability
    consts.gyro.bias.x = 0.648177; % [deg/h]
    consts.gyro.bias.y = 0.682535; % [deg/h]
    consts.gyro.bias.z = 0.592768; % [deg/h]

    consts.gyro.noise = 5e-3; % [dps/sqrt(hz)]

    %% Accelerometer Parameters

    consts.accel.noise = 60e-6; % [g/sqrt(hz)] 




end