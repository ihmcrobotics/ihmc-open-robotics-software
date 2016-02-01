function [qv, qs] = unpackQuaternion(q)
qv = q(1 : 3);
qs = q(4);
end