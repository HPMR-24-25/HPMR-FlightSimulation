function canardInput = constrainMissileAcutationLimits(x_t, canardTargetInput, prevCanardInput, kins, time)
% CONSTRAINMISSILEACTUATIONLIMITS - Constrain Canard Deflection
% Applies structural and physical limits to the canard actuation system

    % Ensure canard commands do not exceed valid actuation range (-maxActuation, maxActuation)
    canardInput.d1 = wrapToPi(canardTargetInput.d1);
    canardInput.d2 = wrapToPi(canardTargetInput.d2);
    canardInput.d3 = wrapToPi(canardTargetInput.d3);
    canardInput.d4 = wrapToPi(canardTargetInput.d4);

    % Simulate actuation speed limitation for each canard
    maxDelta = kins.canard.maxActuationRate * time.dt;

    canardInput.d1 = prevCanardInput.d1 + sign(canardInput.d1 - prevCanardInput.d1) * ...
                     min(maxDelta, abs(canardInput.d1 - prevCanardInput.d1));
    canardInput.d2 = prevCanardInput.d2 + sign(canardInput.d2 - prevCanardInput.d2) * ...
                     min(maxDelta, abs(canardInput.d2 - prevCanardInput.d2));
    canardInput.d3 = prevCanardInput.d3 + sign(canardInput.d3 - prevCanardInput.d3) * ...
                     min(maxDelta, abs(canardInput.d3 - prevCanardInput.d3));
    canardInput.d4 = prevCanardInput.d4 + sign(canardInput.d4 - prevCanardInput.d4) * ...
                     min(maxDelta, abs(canardInput.d4 - prevCanardInput.d4));

    % Constrain canard deflection within symmetric actuation limits
    maxActuation = kins.canard.maxActuation;
    canardInput.d1 = max(-maxActuation, min(maxActuation, canardInput.d1));
    canardInput.d2 = max(-maxActuation, min(maxActuation, canardInput.d2));
    canardInput.d3 = max(-maxActuation, min(maxActuation, canardInput.d3));
    canardInput.d4 = max(-maxActuation, min(maxActuation, canardInput.d4));
end
