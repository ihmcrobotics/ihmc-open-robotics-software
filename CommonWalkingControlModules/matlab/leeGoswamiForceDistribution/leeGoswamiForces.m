
function [F, kdError, ldError] = leeGoswamiForces(kd, ld, m, g, rG, rs, Rs, mus, wk, epsilonf, nSupportVectors)

nFeet = length(mus);
vectorLength = 3;

Beta = zeros(vectorLength, nFeet * nSupportVectors);
Delta = zeros(vectorLength, nFeet * nSupportVectors);
for i = 1 : nFeet
    mu = mus(i);

    RStartIndex = (i - 1) * vectorLength + 1;
    REndIndex = RStartIndex + vectorLength - 1;
    R = Rs(:, RStartIndex : REndIndex);
    r = rs(:, i);

    BetaStartIndex = (i - 1) * nSupportVectors + 1;
    BetaEndIndex = BetaStartIndex + nSupportVectors - 1;
    beta = computeBeta(R, mu, nSupportVectors);
    Beta(:, BetaStartIndex : BetaEndIndex) = beta;

    Delta(:, BetaStartIndex : BetaEndIndex) = tilde(r - rG) * beta;
end

Phi = [Beta;
       wk * Delta;
       epsilonf * eye(nFeet * nSupportVectors)];

xi = [ld - m * g;
      wk * kd
      zeros(nFeet * nSupportVectors, 1)];

cvx_begin
    variable rho(nFeet * nSupportVectors)
    minimize(norm(Phi * rho - xi))
    subject to
        rho >= 0;
cvx_end

F = zeros(vectorLength, nFeet);
for i = 1 : nFeet
    BetaStartIndex = (i - 1) * nSupportVectors + 1;
    BetaEndIndex = BetaStartIndex + nSupportVectors - 1;
    beta = Beta(:, BetaStartIndex : BetaEndIndex);
    
    rhoStartIndex = (i - 1) * nSupportVectors + 1;
    rhoEndIndex = rhoStartIndex + nSupportVectors - 1;

    rhoi = rho(rhoStartIndex : rhoEndIndex);
    f = beta * rhoi;
    F(:, i) = f;
end

ldSol = m * g + Beta * rho;
kdSol = Delta * rho;

ldError = ld - ldSol;
kdError = kd - kdSol;

end