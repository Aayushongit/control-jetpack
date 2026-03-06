function params = jh_refresh_params(params)
if ~isfield(params, 'centerOfMass') || isempty(params.centerOfMass)
    params.centerOfMass = zeros(3, 1);
end

params.centerOfMass = params.centerOfMass(:);
params.weight = params.mass * params.gravity;
params.hover = jh_hover_trim(params);
params.controlEffectiveness = jh_control_effectiveness(params, params.hover);
end
