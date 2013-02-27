function beta = computeBeta(R, mu, nSupportVectors)

angles = linspace(0, 2 * pi, nSupportVectors + 1);
angles = angles(1 : end - 1);

beta = zeros(3, nSupportVectors);
for i = 1 : nSupportVectors
    angle = angles(i);
    beta(:, i) = [mu * cos(angle); mu * sin(angle); 1];
end

beta = R * beta;

end