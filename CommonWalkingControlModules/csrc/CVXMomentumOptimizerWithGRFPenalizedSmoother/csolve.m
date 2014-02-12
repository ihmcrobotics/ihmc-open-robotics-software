% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(A*vd - b, C) + quad_form(Js*vd - ps, Ws) + quad_form(rho, WRho) + quad_form(vd, Lambda) + quad_form(rho - rhoPrevious, WRhoSmoother) + quad_form(rho - rhoPreviousMean, WRhoCoPPenalty))
%   subject to
%     Qrho*rho == A*vd + c
%     rho >= rhoMin
%
% with variables
%      rho  64 x 1
%       vd  38 x 1
%
% and parameters
%        A   6 x 38
%        C   6 x 6    PSD, diagonal
%       Js  38 x 38
%   Lambda  38 x 38   PSD, diagonal
%     Qrho   6 x 64
%     WRho  64 x 64   PSD, diagonal
% WRhoCoPPenalty  64 x 64   PSD, diagonal
% WRhoSmoother  64 x 64   PSD, diagonal
%       Ws  38 x 38   PSD, diagonal
%        b   6 x 1
%        c   6 x 1
%       ps  38 x 1
%   rhoMin  64 x 1    positive
% rhoPrevious  64 x 1    positive
% rhoPreviousMean  64 x 1    positive
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.rhoPreviousMean, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2014-02-11 21:41:08 -0500.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
