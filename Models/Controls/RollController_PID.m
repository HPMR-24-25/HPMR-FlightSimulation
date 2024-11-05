function canardInput = RollController_PID(x, rollCmd, P, I, D, dt)
% RollController_PID - Simple P roll controller
% Roll controller using a P gain
% Inputs:
%   x - Current State
%   rollCmd - [rad] Commanded Roll
%   P - Proportional gain parameter
%   I - Integral Gain Parameter
%   D - Derivative Gain Parameter
% Outputs:
%   canardInput - Struct containing actuation commands for all canards

    eulBuff = quat2eul(x(1:4, :)', 'ZYX')';

    rollBuff = eulBuff(3,:);

    err = rollCmd - rollBuff(2);

    err_d = ((rollCmd - rollBuff(2)) - (rollCmd - rollBuff(1))) / dt;

    err_int = ((rollCmd - rollBuff(2)) - (rollCmd - rollBuff(1))) * dt;

    cmd = err * P + err_int * I + err_d * D;

    % err_int =

    % Roll command is equivalent for all fins
    canardInput.d1 = cmd;
    canardInput.d2 = cmd;
    canardInput.d3 = cmd;
    canardInput.d4 = cmd;
end