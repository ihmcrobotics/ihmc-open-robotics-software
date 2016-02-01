function result=runForceDistributionProblemSolver(feet,env,control, plot)
if nargin < 4
    plot = true;
end
%% combine foot information for solving
rs = [];
Rs = [];
mus = [];
dMin = [];
dMax = [];
epsilonP=[];
for i=1:length(feet)
    rs = [rs,feet(i).r];
    Rs = [Rs,feet(i).R];
    mus = [mus, feet(i).mu];
    params = feet(i).params;
    dMin = [dMin , [params.x.min;params.y.min]];
    dMax = [dMax , [params.x.max;params.y.max]];
    H(i) = params.height;
    tauMax(i) = feet(i).tauMax;
    epsilonP = [epsilonP,params.epsilonCoP, params.epsilonCoP, params.epsilonTauN];
end

%% unpack environment
rG = env.rG;
m = env.m;
g = env.g;

%% unpack control
ld = control.ld;
kd = control.kd;
wk = control.wk;
epsilonf = control.epsilonf;

%% solver specific parameters

nSupportVectors = 4;

%% Run the solver
[F, kdError, ldError] = leeGoswamiForces(kd, ld, m, g, rG, rs, Rs, mus, wk, epsilonf, nSupportVectors);
[cops, tauns, kdError] = leeGoswamiCops(kdError, F, rs, Rs, dMin, dMax, tauMax, H, epsilonP);
if plot
    visualize(rG, rs, Rs, dMin, dMax, H, F, cops);
end

nFeet = size(F, 2);

kdErrorCheck = kd;
ldErrorCheck = ld - m * g;
for i = 1 : nFeet
    cop = cops(:, i);
    f = F(:, i);
    tauN = tauns(:, i);
    
    kdErrorCheck = kdErrorCheck - cross((cop - rG),f) - tauN;
    ldErrorCheck = ldErrorCheck - f;
    feet(i).F = f;
    feet(i).tauN = tauN;
    feet(i).cop = cop;
end

assertEpsilonEquals('kd error should be accurately checked',kdErrorCheck, kdError,1e-6);
assertEpsilonEquals('ld error should be accurately checked',ldErrorCheck, ldError,1e-6);
result.feet = feet;
result.ldError = ldError;
result.kdError = kdError;

result.kdEps = norm(result.kdError);
result.ldEps = norm(result.ldError);
end