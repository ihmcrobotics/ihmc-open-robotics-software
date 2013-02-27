function deltaQ = deltaPhiToDeltaQ(deltaPhi)
deltaPhiNorm = norm(deltaPhi);
if deltaPhiNorm > eps
    argument = 0.5 * deltaPhiNorm;
    qv = sin(argument) * deltaPhi / deltaPhiNorm;
    qs = cos(argument);
    deltaQ = [qv; qs];
else
    deltaQ = [0; 0; 0; 1];
end
end