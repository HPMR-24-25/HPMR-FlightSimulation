function eul = hpmr_quat2eul(q)
    % Ensure q is a 4xN matrix
    if size(q,1) ~= 4
        error('Input quaternion buffer must be a 4xN matrix.');
    end

    % Number of quaternions
    N = size(q,2);

    % Preallocate output
    eul = zeros(3, N);

    % Compute Euler angles for each quaternion in the buffer
    for i = 1:N
        eul(:, i) = [
            atan2(2*(q(1,i)*q(2,i) + q(3,i)*q(4,i)), 1 - 2*(q(2,i)^2 + q(3,i)^2));
            (-pi/2) + 2*atan2(sqrt(1 + 2*(q(1,i)*q(3,i) - q(2,i)*q(4,i))), sqrt(1 - 2*(q(1,i)*q(3,i) - q(2,i)*q(4,i))));
            atan2(2*(q(1,i)*q(4,i) + q(2,i)*q(3,i)), 1 - 2*(q(3,i)^2 + q(4,i)^2))
        ];
    end
end
