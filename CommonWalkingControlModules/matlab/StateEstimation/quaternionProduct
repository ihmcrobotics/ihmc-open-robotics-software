function r = quaternionProduct(q, p)
[qv, qs] = unpackQuaternion(q);
[pv, ps] = unpackQuaternion(p);

rv = qs * pv + ps * qv + cross(qv, pv);
rs = qs * ps - dot(qv, pv);
r = packQuaternion(rv, rs);
end