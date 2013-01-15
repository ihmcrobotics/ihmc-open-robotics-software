ld = [0.1; -0.1; 0.1];
kd = [0.0; 0.0; 0.0];

m = 100;
g = [0; 0; -9.81];

r0 = [0; 0; 0;];
r1 = [0.5; 0.5; 0.1];
r2 = [0.25; 0.5; 0.8];

rG = [0.25; 0.25; 1.0];

rs = [r0, r1, r2];
Rs = [rotX(-0.2) * rotY(0.1), rotX(0.1) * rotY(-0.3), rotX(0.3) * rotY(-0.2)];
mus = [0.6; 0.6; 0.4];

dMin = [-0.07, -0.07, -0.07;
        -0.06, -0.06, -0.06];
    
dMax = [0.18, 0.18, 0.18;
        0.06, 0.06, 0.06];
H = [0.084, 0.084, 0.084];
    
tauMax = [10, 10, 3];

wk = 0.1;
epsilonf = 0.01;
epsilonCoP = 0.01;
epsilonTauN = 0.0001; 
epsilonP = [epsilonCoP, epsilonCoP, epsilonTauN, epsilonCoP, epsilonCoP, epsilonTauN, epsilonCoP, epsilonCoP, epsilonTauN];
nSupportVectors = 4;

[F, kdError, ldError] = leeGoswamiForces(kd, ld, m, g, rG, rs, Rs, mus, wk, epsilonf, nSupportVectors);
[cops, tauns, kdError] = leeGoswamiCops(kdError, F, rs, Rs, dMin, dMax, tauMax, H, epsilonP);
visualize(rG, rs, Rs, dMin, dMax, H, F, cops);

nFeet = size(F, 2);

kdErrorCheck = kd;
ldErrorCheck = ld - m * g;
for i = 1 : nFeet
    cop = cops(:, i);
    f = F(:, i);
    tauN = tauns(:, i);
    
    kdErrorCheck = kdErrorCheck - cross((cop - rG), f) - tauN;
    ldErrorCheck = ldErrorCheck - f;
end
