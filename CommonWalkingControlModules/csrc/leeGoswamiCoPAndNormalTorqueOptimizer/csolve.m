% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(Psi_k*eta - kappa_k, eye(3)) + quad_form(eta - eta_d, epsilon))
%   subject to
%     etamin <= eta
%     eta <= etamax
%
% with variables
%      eta   6 x 1
%
% and parameters
%    Psi_k   3 x 6
%  epsilon   6 x 6    PSD, diagonal
%    eta_d   6 x 1
%   etamax   6 x 1
%   etamin   6 x 1
%  kappa_k   3 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.Psi_k, ..., params.kappa_k, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2013-01-17 22:52:37 +0000.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
