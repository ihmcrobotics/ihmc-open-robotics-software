function [friction] = asymmetricCoulombViscous(velocity, parameters)
% Friction force based on coulomb + viscous model.
% Expected inputs:
% * joint velocity
% * 'parameters' is a structure containing:
%   - positive Coulomb coefficient
%   - positive Viscous coeffifient
%   - negative Coulomb coefficient
%   - negative Viscous coeffifient
if velocity >= 0
    friction = sign(velocity) * parameters.pCoulomb + ...
               velocity * parameters.pViscous;
else
    friction = sign(velocity) * parameters.nCoulomb + ...
               velocity * parameters.nViscous;
end