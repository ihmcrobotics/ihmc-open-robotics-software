function [friction] = asymmetricPressureBased(velocity, posPressure, negPressure, parameters)
% Friction force pressure based for hydraulic actuators.
% Expected inputs:
% * joint velocity
% * positive pressure
% * negative pressure
% * 'parameters' is a structure containing:
%   - x1 coefficient
%   - x2 coeffifient
%   - x3 coefficient
%   - x4 coeffifient
%   - x5 coeffifient

friction = parameters.x1 * exp(parameters.x2 * velocity) + ...
           parameters.x3 * (posPressure - negPressure) + ...
           parameters.x4 * negPressure + parameters.x5 * velocity;