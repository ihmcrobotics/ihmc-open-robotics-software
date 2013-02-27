function q = normalizeQuaternion(q)
qNorm = norm(q);
if abs(qNorm - 1 > 1e-1)
    error(['qNorm is ' num2str(qNorm)]);
end
q = q / qNorm;
end