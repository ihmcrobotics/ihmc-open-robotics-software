ex = [1; 0; 0];
ey = [0; 1; 0];
ez = [0; 0; 1];

S0 = [ex, ey];
S1 = [ex, ey];

en0 = ez;
en1 = ez;

f0 = [1; 0; 0]; %rand(3, 1);
f1 = [1; 0; 0]; rand(3, 1);

tau = [0; 0; 1]; %rand(3, 1);

min0 = [-0.1; -0.5];
max0 = [0; -0.4];

min1 = [0.2; 0.4];
max1 = [0.3; 0.5];

cvx_begin
    variable cop0(2)
    variable m0(1)
    variable cop1(2)
    variable m1(1)

    minimize(0)
    subject to
        -tilde(f0) * S0 * cop0 + en0 * m0 - tilde(f1) * S1 * cop1 + en1 * m1 == tau;
        min0 <= cop0 <= max0;
        min1 <= cop1 <= max1;
cvx_end

error = -tilde(f0) * S0 * cop0 + en0 * m0 - tilde(f1) * S1 * cop1 + en1 * m1 - tau;