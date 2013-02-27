function q = packQuaternion(qv, qs)
if length(qv) ~= 3
    error('length(qv) ~= 3');
end
q = [qv; qs];
end