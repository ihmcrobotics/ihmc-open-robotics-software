
ld = [0.1; -0.1; 0.1];
kd = [0.0; 0.0; 0.0];

m = 100;
g = [0; 0; -9.81];
wk = 0.1;
epsilonf = 0.01;

r0 = [0; 0; 0;];
r1 = [0.5; 0.5; 0.1];
rG = [0.25; 0.25; 1.0];

cvx_begin
    variable f0(3)
    variable f1(3)

    minimize(norm(ld - m * g - f0 - f1) + wk * norm(kd - tilde(r0 - rG) * f0 - tilde(r1 - rG) * f1) + epsilonf * (norm(f0) + norm(f1)))
cvx_end