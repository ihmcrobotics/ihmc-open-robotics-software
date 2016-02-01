% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(Phi*rho - xi, eye(6)) + epsilon_f*epsilon_f*quad_form(rho, eye(8)))
%   subject to
%     rho >= 0
%
% with variables
%      rho   8 x 1
%
% and parameters
%      Phi   6 x 8
% epsilon_f   1 x 1    positive
%       xi   6 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.Phi, ..., params.xi, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2013-01-17 15:52:18 +0000.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
