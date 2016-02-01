function deltaPhi = deltaQToDeltaPhi(deltaQ)
[qv, qs] = unpackQuaternion(deltaQ);
normVSquared = dot(qv, qv);
if normVSquared > 1e-12
    normV = sqrt(normVSquared);
    angle = 2 * atan2(normV, qs);
    deltaPhi = qv / normV * angle;
else
    deltaPhi = [0; 0; 0];
end
end