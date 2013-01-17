function [kdEps,ldEpsr]=runMonopodTest(footTiltTheta,wk,epsilonf, plot)
if nargin<4
    plot = false;
end
[feet,env,control]=generateUnsolvableMonopod(footTiltTheta,wk,epsilonf,0.25);
% the trick is to make the weighting for kd low!
result = runForceDistributionProblemSolver(feet,env,control,plot);
kdEps = norm(result.kdError);
ldEpsr = norm(result.ldError);
end