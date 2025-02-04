function Torques = Controller_Lyapunov(x, cmd, P, I, D, dt)
% Controller_Lyapunov - Lyapunov attitude controller
% Roll controller using a P and D gains
% Inputs:
%   x - Current State
%   cmd - [rad] Commanded Roll, Pitch, and Yaw
%   P - Proportional gain parameter
%   I - Integral Gain Parameter
%   D - Derivative Gain Parameter
% Outputs:
%   Torques - Three Canard Torques to achieve commanded attitude command

%%
% Original
% q(1) = x(1);
% q(2) = x(2);
% q(3) = x(3);
% q(4) = x(4);

% Swapped
q = [x(2); x(3); x(4); x(1)];

w = [x(11); x(12); x(13)];

roll_cmd = cmd(1);
pitch_cmd = cmd(2);
yaw_cmd = cmd(3);

% not sure if this function works for quaternion set up
qc = eul2quat(roll_cmd, pitch_cmd, yaw_cmd)';

dq = Qmult(q,Qinv(qc));

L = -P*sign(dq(4))*dq(1:3)-D*(1-dq(1:3)'*dq(1:3))*w;

Torques = L;

    function quadprod = Qmult(p, q)
        q13 = q(1:3);
        p13 = p(1:3);
        q4 = q(4);
        p4 = p(4);

        quadprod1 = q4*p13 + p4*q13 - cross(p13, q13);
        quadprod2 = p4*q4 - dot(p13, q13);

        quadprod = [quadprod1; quadprod2];
    end

    function p = Qinv(q)
        q1 = q(1);
        q2 = q(2);
        q3 = q(3);
        q4 = q(4);

        q_mag = q1^2+q2^2+q3^2+q4^2;
        q_star = [-q1; -q2; -q3; q4];

        p = q_star/(q_mag^2);
    end
end