kd = [0.5043481119806846, 77.69149873772825, -0.011788812328126716]';
ld = [-50.785684515236305, 0.3938074403020791, 422.58931702091854]';

rG = [0; 0; 0];

g = [0; 0; -9.81];

% kdTau = -[0.45393188350765756, 69.32261578413751, -0.010284987599418899]';
% 
% f0 = [-25.375479464857055, 0.1968836254516937, 211.2962288490925]';
% f1 = [-25.346581637892452, 0.19688362529125314, 211.28295661238076]';
% F = [f0, f1];

r0 = [-0.0808601004638885, 0.08928054667452709, -0.97628197807719]';
r1 = [-0.0808601004638885, -0.0887194533254729, -0.97628197807719]';
rs = [r0, r1];

R0 = eye(3);
R1 = eye(3);
Rs = [R0, R1];

mus = [0.5, 0.5];

dMin0 = [-0.07; -0.06];
dMin1 = dMin0;
dMin = [dMin0, dMin1];

dMax0 = [0.18; 0.06];
dMax1 = dMax0;
dMax = [dMax0, dMax1];

tauMax = [105.64811442454625, 105.64811442454625];

H = [0, 0];

wk = 0.1;
epsilonf = 0.01;
nSupportVectors = 4;

epsilonCoP = 0.01;
epsilonTauN = 0.001;
epsilonP = [epsilonCoP, epsilonCoP, epsilonTauN, epsilonCoP, epsilonCoP, epsilonTauN];


[F, kdError, ldError] = leeGoswamiForces(kd, ld, 0, g, rG, rs, Rs, mus, wk, epsilonf, nSupportVectors);
[cops, tauns, kdError] = leeGoswamiCops(kdError, F, rs, Rs, dMin, dMax, tauMax, H, epsilonP);