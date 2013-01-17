function params = generateLineFootParams()
params.height = 0.00;
eps = 0;
params.x.max = 0.0+eps;
params.x.min = 0.0-eps;
params.y.max = 0.06;
params.y.min = -0.06;
params.epsilonCoP = 0.01;
params.epsilonTauN = 0.0001;
end