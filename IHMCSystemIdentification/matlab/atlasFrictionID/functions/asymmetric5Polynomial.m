function [friction] = asymmetric5Polynomial(velocity, parameters)
% Friction force based on 4th order polynomial.
% Expected inputs:
% * joint velocity
% * 'parameters' is a structure containing:
%   - positive x0, x1, x2, x3, x3
%   - negative  x0, x1, x2, x3, x3

if velocity >= 0
    friction = parameters.px0 + parameters.px1 * velocity + parameters.px2 * velocity^2 + parameters.px3 * velocity^3 + parameters.px4 * velocity^4 + parameters.px5 * velocity^5;
else
    friction = parameters.nx0 + parameters.nx1 * velocity + parameters.nx2 * velocity^2 + parameters.nx3 * velocity^3 + parameters.nx4 * velocity^4 + parameters.nx5 * velocity^5;
end