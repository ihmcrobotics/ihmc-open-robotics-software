% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(A*vd - b, C) + quad_form(Js*vd - ps, Ws) + wRho*quad_form(rho, eye(40)) + quad_form(vd, Lambda))
%   subject to
%     Qrho*rho + Qphi*phi == A*vd + c
%     N'*vd == z
%     rho >= rhoMin
%     phiMin <= phi
%     phi <= phiMax
%
% with variables
%      phi  10 x 1
%      rho  40 x 1
%       vd  34 x 1
%
% and parameters
%        A   6 x 34
%        C   6 x 6    PSD, diagonal
%       Js  34 x 34
%   Lambda  34 x 34   PSD, diagonal
%        N  34 x 4
%     Qphi   6 x 10
%     Qrho   6 x 40
%       Ws  34 x 34   PSD, diagonal
%        b   6 x 1
%        c   6 x 1
%   phiMax  10 x 1    positive
%   phiMin  10 x 1    negative
%       ps  34 x 1
%   rhoMin  40 x 1    positive
%     wRho   1 x 1    positive
%        z   4 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.z, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2013-05-07 16:39:31 -0400.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
