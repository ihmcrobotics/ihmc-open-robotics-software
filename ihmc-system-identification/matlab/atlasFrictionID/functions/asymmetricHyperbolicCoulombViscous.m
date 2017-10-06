function [friction] = asymmetricHyperbolicCoulombViscous(velocity, parameters)
% Friction force based on coulomb + viscous model.
% Expected inputs:
% * joint velocity
% * 'parameters' is a structure containing:
%   - positive Coulomb coefficient
%   - positive hyperbolic coefficient
%   - positive Viscous coeffifient
%   - negative Coulomb coefficient
%   - negative hyperbolic coefficient
%   - negative Viscous coeffifient
if velocity >= 0
    friction = sign(velocity) * parameters.pCoulomb + parameters.pHyperbolicCoeff * tanh(velocity) +...
               velocity * parameters.pViscous;
else
    friction = sign(velocity) * parameters.nCoulomb + parameters.nHyperbolicCoeff * tanh(velocity) +...
               velocity * parameters.nViscous;
end