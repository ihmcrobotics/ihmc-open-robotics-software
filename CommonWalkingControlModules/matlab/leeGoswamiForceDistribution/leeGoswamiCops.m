function [cops, tauns, kdError] = leeGoswamiCops(kdTau, F, rs, Rs, dMin, dMax, tauMax, H, epsilonP)

nFeet = size(F, 2);

etaMin = zeros(3 * nFeet, 1);
etaMax = zeros(3 * nFeet, 1);

vectorLength = 3;
etaiLength = 3;
for i = 1 : nFeet
    startIndex = (i - 1) * etaiLength + 1;
    endIndex = startIndex + etaiLength - 1;
    etaMini = [dMin(:, i); -tauMax(i)];
    etaMaxi = [dMax(:, i);  tauMax(i)];
    etaMin(startIndex : endIndex) = etaMini;
    etaMax(startIndex : endIndex) = etaMaxi;
end

etad = (etaMin + etaMax) / 2;

Psik = zeros(vectorLength, etaiLength * nFeet);
kappak = kdTau;
for i = 1 : nFeet
    f = F(:, i);

    RStartIndex = (i - 1) * vectorLength + 1;
    REndIndex = RStartIndex + vectorLength - 1;
    R = Rs(:, RStartIndex : REndIndex);

%     fb = R' * f;
%     col0 = -R(:, 2) * fb(3) + R(:, 3) * fb(2);
%     col1 = R(:, 1) * fb(3) - R(:, 3) * fb(1);
%     col2 = R(:, 3);
% 
%     Psiki = [col0, col1, col2];
    A = -tilde(f) * R;
    Psiki = [A(:, 1:2) R(:, 3)];
    Psik(:, RStartIndex : REndIndex) = Psiki;

    h = H(i);
%     kappaki = h * (R(:, 2) * fb(1) - R(:, 1) * fb(2))
    kappaki = A(:, 3) * h;
    kappak = kappak + kappaki;
end


epsilon = diag(epsilonP);
Psi = [Psik;
       epsilon];

kappa = [kappak;
         epsilon * etad];

cvx_begin
    variable eta(nFeet * etaiLength)
    minimize(norm(Psi * eta - kappa))
    subject to
        etaMin <= eta <= etaMax;
cvx_end

cops = zeros(vectorLength, nFeet);
tauns = zeros(vectorLength, nFeet);
for i = 1 : nFeet
    copStartIndex = (i - 1) * etaiLength + 1;
    copEndIndex = copStartIndex + 1;
    taunIndex = copEndIndex + 1;

    h = H(i);
    
    RStartIndex = (i - 1) * vectorLength + 1;
    REndIndex = RStartIndex + vectorLength - 1;
    R = Rs(:, RStartIndex : REndIndex);
    
    r = rs(:, i);

    copb = [eta(copStartIndex : copEndIndex); -h];

    cop = R * copb + r;
    cops(:, i) = cop;
    
    taun = R(:, 3) * eta(taunIndex);
    tauns(:, i) = taun;
end

error = Psi * eta - kappa;
kdError = error(1 : vectorLength);

end
