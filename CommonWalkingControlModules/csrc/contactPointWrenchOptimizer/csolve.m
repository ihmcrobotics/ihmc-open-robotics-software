% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(A*rho - W, C) + epsilon*quad_form(rho, eye(64)))
%   subject to
%     rho >= rhomin
%
% with variables
%      rho  64 x 1
%
% and parameters
%        A   6 x 64
%        C   6 x 6    PSD, diagonal
%        W   6 x 1
%  epsilon   1 x 1    positive
%   rhomin  64 x 1    positive
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.rhomin, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2013-03-11 17:02:06 -0400.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
