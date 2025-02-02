function canardInput = Controller_Lyapunov(x, cmd, P, I, D, dt)
% Controller_Lyapunov - Lyapunov attitude controller
% Roll controller using a P and D gains
% Inputs:
%   x - Current State
%   cmd - [rad] Commanded Roll, Pitch, and Yaw
%   P - Proportional gain parameter
%   I - Integral Gain Parameter
%   D - Derivative Gain Parameter
% Outputs:
%   canardInput - Struct containing actuation commands for all canards

%%
% Original
% q(1) = x(1);
% q(2) = x(2);
% q(3) = x(3);
% q(4) = x(4);

% x = [0; 1; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
% cmd = [deg2rad(35); 0; 0];

% Swapped
q = [x(2); x(3); x(4); x(1)];

w = [x(11); x(12); x(13)];

roll_cmd = cmd(1);
pitch_cmd = cmd(2);
yaw_cmd = cmd(3);

qc = eul2quat(roll_cmd, pitch_cmd, yaw_cmd)';

dq = Qmult(q,Qinv(qc));

%%
% p = Qinv(qc);
%         q13 = q(1:3);
%         p13 = p(1:3);
%         q4 = q(4);
%         p4 = p(4);
% 
%         quadprod1 = q4*p13 + (p4*q13) - cross(p13, q13);
%         quadprod2 = p4*q4 - dot(p13, q13);
% 
%         dq = [quadprod1; quadprod2];

%%
% P = 0.4;
% D = 0;

L = -P*sign(dq(4))*dq(1:3)-D*(1-dq(1:3)'*dq(1:3))*w;

canardInput = L;



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
        q_mag = q(1)^2+q(2)^2+q(3)^2+q(4)^2;
        p = [-q(1)/q_mag; -q(2)/q_mag; -q(3)/q_mag; q(4)/q_mag];
    end
end









% from before
%     eulBuff = quat2eul(x(1:4, :)', 'ZYX')';
% 
%     rollBuff = eulBuff(3,:);
% 
%     err = rollCmd - rollBuff(2);
% 
%     err_d = ((rollCmd - rollBuff(2)) - (rollCmd - rollBuff(1))) / dt;
% 
%     err_int = ((rollCmd - rollBuff(2)) - (rollCmd - rollBuff(1))) * dt;
% 
%     cmd = err * P + err_int * I + err_d * D;
% 
%     err_int =
% 
%     Roll command is equivalent for all fins
%     canardInput.d1 = cmd;
%     canardInput.d2 = cmd;
%     canardInput.d3 = cmd;
%     canardInput.d4 = cmd;