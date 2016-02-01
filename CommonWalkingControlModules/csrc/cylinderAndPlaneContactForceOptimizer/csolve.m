% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(Qrho*rho + Qphi*phi - c, C) + wRho*quad_form(rho, eye(40)) + wPhi*quad_form(phi, eye(10)))
%   subject to
%     rho >= rhoMin
%     phiMin <= phi
%     phi <= phiMax
%
% with variables
%      phi  10 x 1
%      rho  40 x 1
%
% and parameters
%        C   6 x 6    PSD, diagonal
%     Qphi   6 x 10
%     Qrho   6 x 40
%        c   6 x 1
%   phiMax  10 x 1    positive
%   phiMin  10 x 1    negative
%   rhoMin  40 x 1    positive
%     wPhi   1 x 1    positive
%     wRho   1 x 1    positive
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.C, ..., params.wRho, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2013-05-08 14:35:33 -0400.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
