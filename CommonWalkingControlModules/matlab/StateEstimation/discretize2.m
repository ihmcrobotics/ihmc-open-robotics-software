function [Ad, Bd, Qd, Rd] = discretize2(A, B, Q, R, deltaT)
n = size(A, 1);
m = size(B, 2);
C = [-A', Q, zeros(n, m);
     zeros(n, n), A, B;
     zeros(m, n), zeros(m, n), zeros(m, m)];

expC = expm(C * deltaT);
% nApprox = 3;
% expC = expmApprox(C * deltaT, nApprox);
F3 = expC(n + 1 : 2 * n, n + 1 : 2 * n);
G2 = expC(1 : n, n + 1 : 2 * n);
G3 = expC(n + 1 : 2 * n, 2 * n + 1 : end);

Ad = F3;
Bd = F3 * G3;
Qd = F3' * G2;

Rd = R / deltaT;
end

function A = expmApprox(M, n)
A = zeros(size(M));
factorial = 1;
MPower = eye(size(M, 1));
for i = 1 : n
    A = A + 1 / factorial * MPower;
    factorial = factorial * i;
    MPower = MPower * M;
end

end
