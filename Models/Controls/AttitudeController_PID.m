function [canardInput, T, err] = AttitudeController_PID(x, desiredQuat, GAIN_1, GAIN_2, dt, kins, inds, AeroModel)

    % Extract current quaternion and angular velocity
    q = x(inds.q, 1);    % Current attitude quaternion
    omega = x(inds.w_ib, 1); % Angular velocity in body frame

    % Compute quaternion error
    q_err = quatmultiply(quatinv(desiredQuat'), q'); % Error quaternion

    % Convert quaternion error to rotation vector (small angle approximation)
    q_vec = q_err(2:4)'; % Extract vector part of quaternion
    err = 2 * q_vec .* sign(q_err(1)); % Ensure shortest rotation path

    % Integral term with anti-windup
    persistent err_int
    if isempty(err_int)
        err_int = [0; 0; 0];
    end
    err_int = max(-1, min(1, err_int + err * dt)); % Clamping integral to prevent windup

    % Compute control torques (P + I + D)
    T_x = GAIN_1(1) * err(1) + GAIN_1(2) * err_int(1) - GAIN_1(3) * omega(1);
    T_y = GAIN_2(1) * err(2) + GAIN_2(2) * err_int(2) - GAIN_2(3) * omega(2);
    T_z = GAIN_2(1) * err(3) + GAIN_2(2) * err_int(3) - GAIN_2(3) * omega(3);

    % Store torque outputs
    T = [T_x; T_y; T_z];

    % --- Atmospheric Model ---
    lla = ecef2lla(x(inds.pos), "wgs84");
    alt = lla(3);
    AtmosphericModel(alt);

    % --- Aerodynamic Parameters ---
    rho_inf = AtmosphericModel.rho_sl;
    v_inf = norm(x(inds.vel, 1));
    q_inf = 0.5 * rho_inf * v_inf^2;

    C_p = (kins.diameter / 2) + (kins.canard.height / 2);

    % --- Control Allocation Matrix ---
    A = [
        C_p -C_p C_p -C_p;
        0 kins.x_cp 0 -kins.x_cp;
        -kins.x_cp 0 kins.x_cp 0;
    ];

    % Compute required control input
    b = (1 / (q_inf * kins.canard.S * AeroModel.canard.CL_delta)) * [T_x; T_y; T_z];

    % Solve for canard deflections
    cmd = pinv(A) * b;

    % Limit canard deflections
    maxCanardDeflection = kins.canard.maxActuation;
    canardInput.d1 = cmd(1);
    canardInput.d2 = cmd(2);
    canardInput.d3 = cmd(3);
    canardInput.d4 = cmd(4);
    % canardInput.d1 = max(-maxCanardDeflection, min(maxCanardDeflection, cmd(1)));
    % canardInput.d2 = max(-maxCanardDeflection, min(maxCanardDeflection, cmd(2)));
    % canardInput.d3 = max(-maxCanardDeflection, min(maxCanardDeflection, cmd(3)));
    % canardInput.d4 = max(-maxCanardDeflection, min(maxCanardDeflection, cmd(4)));

end