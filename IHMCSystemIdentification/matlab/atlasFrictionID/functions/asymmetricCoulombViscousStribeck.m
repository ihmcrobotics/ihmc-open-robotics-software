function [friction] = asymmetricCoulombViscousStribeck(velocity, parameters)
% Friction force based on coulomb + viscous + Stribeck model.
% Expected inputs:
% * joint velocity
% * 'parameters' is a structure containing:
%   - positiveSigma
%   - positiveFc0
%   - positiveFs0
%   - positiveCs
%   - negativeSigma
%   - negativeFc0
%   - negativeFs0
%   - negativeCs

if velocity >= 0
    friction = parameters.positiveSigma * velocity + sign(velocity) * (parameters.positiveFc0 + ...
               parameters.positiveFs0 * exp(-(abs(velocity)/parameters.positiveCs)));
else
    friction = parameters.negativeSigma * velocity + sign(velocity) * (parameters.negativeFc0 + ...
               parameters.negativeFs0 * exp(-(abs(velocity)/parameters.negativeCs)));
end