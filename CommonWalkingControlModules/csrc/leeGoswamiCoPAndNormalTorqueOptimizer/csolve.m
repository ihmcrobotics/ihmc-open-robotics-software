% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(Psi_k_1*eta_1 - kappa_k_1, eye(3)) + quad_form(epsilon*(eta_1 - eta_d_1), eye(3)) + quad_form(Psi_k_2*eta_2 - kappa_k_2, eye(3)) + quad_form(epsilon*(eta_2 - eta_d_2), eye(3)))
%   subject to
%     etamin_1 <= eta_1
%     etamin_2 <= eta_2
%     eta_1 <= etamax_1
%     eta_2 <= etamax_2
%
% with variables
%    eta_1   3 x 1
%    eta_2   3 x 1
%
% and parameters
%  Psi_k_1   3 x 3
%  Psi_k_2   3 x 3
%  epsilon   3 x 3    PSD
%  eta_d_1   3 x 1
%  eta_d_2   3 x 1
% etamax_1   3 x 1
% etamax_2   3 x 1
% etamin_1   3 x 1
% etamin_2   3 x 1
% kappa_k_1   3 x 1
% kappa_k_2   3 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.Psi_k_1, ..., params.kappa_k_2, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2013-01-17 22:23:42 +0000.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
