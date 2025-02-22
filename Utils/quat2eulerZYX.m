function eulerAngles = quat2eulerZYX(quatList)
    % Input: quatList - Nx4 matrix, where each row is a quaternion [w, x, y, z]
    % Output: eulerAngles - Nx3 matrix, where each row is [roll, pitch, yaw] in degrees

    % Number of quaternions in the list
    numQuats = size(quatList, 1);
    
    % Initialize output matrix (Nx3) for roll, pitch, and yaw
    eulerAngles = zeros(numQuats, 3);
    
    % Loop through each quaternion
    for i = 1:numQuats
        q = quatList(i, :);
        q_w = q(1);
        q_x = q(2);
        q_y = q(3);
        q_z = q(4);
        
        % Compute Yaw (ψ) - Rotation around Z-axis
        yaw = atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y^2 + q_z^2));

        % Compute Pitch (θ) - Rotation around Y-axis
        pitch = asin(2 * (q_w * q_y - q_z * q_x));

        % Compute Roll (φ) - Rotation around X-axis
        roll = atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x^2 + q_y^2));

        % Convert to degrees
        eulerAngles(i, :) = [roll, pitch, yaw];
    end
end
