function canardInput = constrainMissileAcutationLimits(x_t, canardTargetInput, prevCanardInput, kins, time)
% CONSTRAINMISSILEACTUATIONLIMTS - Constrain Canard Deflection
% Applys structural and physical limits to the canard acutation system

    % Ensure canard commands do not exceed 360 degrees
        canardInput.d1 = mod(canardTargetInput.d1, 2*pi);
        canardInput.d2 = mod(canardTargetInput.d2, 2*pi);
        canardInput.d3 = mod(canardTargetInput.d3, 2*pi);
        canardInput.d4 = mod(canardTargetInput.d4, 2*pi);

        % Simulate actuation speed limitation for each canard
        canardInput.d1 = prevCanardInput.d1 + sign(canardInput.d1 - prevCanardInput.d1) * ...
                         min(kins.canard.maxActuationRate * time.dt, abs(canardTargetInput.d1 - prevCanardInput.d1));
        canardInput.d2 = prevCanardInput.d2 + sign(canardInput.d2 - prevCanardInput.d2) * ...
                         min(kins.canard.maxActuationRate * time.dt, abs(canardTargetInput.d2 - prevCanardInput.d2));
        canardInput.d3 = prevCanardInput.d3 + sign(canardInput.d3 - prevCanardInput.d3) * ...
                         min(kins.canard.maxActuationRate * time.dt, abs(canardTargetInput.d3 - prevCanardInput.d3));
        canardInput.d4 = prevCanardInput.d4 + sign(canardInput.d4 - prevCanardInput.d4) * ...
                         min(kins.canard.maxActuationRate * time.dt, abs(canardTargetInput.d4 - prevCanardInput.d4));

        canardInput.d1 = min(kins.canard.maxActuation, canardInput.d1);
        canardInput.d2 = min(kins.canard.maxActuation, canardInput.d2);
        canardInput.d3 = min(kins.canard.maxActuation, canardInput.d3);
        canardInput.d4 = min(kins.canard.maxActuation, canardInput.d4);
end