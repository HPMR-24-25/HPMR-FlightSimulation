function canardInput = constrainMissileActuationLimits(x_t, canardTargetInput, prevCanardInput, kins, time)
% CONSTRAINMISSILEACTUATIONLIMITS - Constrain Canard Deflection
% Simulates physical constraints such as actuation rate and deflection limits

    % Maximum actuation range
    maxActuation = kins.canard.maxActuation;

    % Ensure target input stays within max deflection limits
    canardTargetInput.d1 = max(-maxActuation, min(maxActuation, canardTargetInput.d1));
    canardTargetInput.d2 = max(-maxActuation, min(maxActuation, canardTargetInput.d2));
    canardTargetInput.d3 = max(-maxActuation, min(maxActuation, canardTargetInput.d3));
    canardTargetInput.d4 = max(-maxActuation, min(maxActuation, canardTargetInput.d4));

    % Compute maximum allowed change per time step
    maxDelta = kins.canard.maxActuationRate * time.dt;

    % Apply actuation speed limitation
    canardInput.d1 = prevCanardInput.d1 + sign(canardTargetInput.d1 - prevCanardInput.d1) * ...
                     min(maxDelta, abs(canardTargetInput.d1 - prevCanardInput.d1));
    canardInput.d2 = prevCanardInput.d2 + sign(canardTargetInput.d2 - prevCanardInput.d2) * ...
                     min(maxDelta, abs(canardTargetInput.d2 - prevCanardInput.d2));
    canardInput.d3 = prevCanardInput.d3 + sign(canardTargetInput.d3 - prevCanardInput.d3) * ...
                     min(maxDelta, abs(canardTargetInput.d3 - prevCanardInput.d3));
    canardInput.d4 = prevCanardInput.d4 + sign(canardTargetInput.d4 - prevCanardInput.d4) * ...
                     min(maxDelta, abs(canardTargetInput.d4 - prevCanardInput.d4));

    % Ensure final values stay within limits
    canardInput.d1 = max(-maxActuation, min(maxActuation, canardInput.d1));
    canardInput.d2 = max(-maxActuation, min(maxActuation, canardInput.d2));
    canardInput.d3 = max(-maxActuation, min(maxActuation, canardInput.d3));
    canardInput.d4 = max(-maxActuation, min(maxActuation, canardInput.d4));
end
