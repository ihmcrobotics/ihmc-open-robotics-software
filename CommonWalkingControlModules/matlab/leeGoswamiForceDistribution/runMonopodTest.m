function [kdEps,ldEpsr]=runMonopodTest(footTiltTheta,wk,epsilonf, plot, yd, mu)
if nargin<4
    plot = false;
end
if nargin<5
    yd = 0.25;
end
if nargin<6
    mu = 0.04;
end
[feet,env,control]=generateUnsolvableMonopod(footTiltTheta,wk,epsilonf,yd, mu);
% the trick is to make the weighting for kd low!
result = runForceDistributionProblemSolver(feet,env,control,plot);
kdEps = norm(result.kdError);
ldEpsr = norm(result.ldError);
end