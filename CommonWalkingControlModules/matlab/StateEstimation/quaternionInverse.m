function qInv = quaternionInverse(q)
[qv, qs] = unpackQuaternion(q);
qInv = [-qv; qs];
% qInv = normalizeQuaternion(qInv);
end