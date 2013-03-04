function state = kalmanFilterQuaternion(state, input, measurements, covariances, dt)

% unpack
qHat = state.qHat;
omegaHat = state.omegaHat;
bHat = state.bHat;
P = state.P;

omegad = input.omegad;
QPhi = covariances.QPhi;
QOmega = covariances.QOmega;
QB = covariances.QB;

zQ = measurements.zQ;
zOmega = measurements.zOmega;
RPhi = covariances.RPhi;
ROmega = covariances.ROmega;

% create system
[A, B] = createContinuousTimeSystem(omegaHat);
QContinuous = blkdiag(QPhi, QOmega, QB);
RContinuous = blkdiag(RPhi, ROmega);
BW = eye(9);
[F, G, Q, R] = discretize(A, B, BW, QContinuous, RContinuous, dt);
H = [eye(3), zeros(3, 6);
     zeros(3, 3), eye(3), eye(3)];

% a priori state
% omegaHat is expressed in body frame, so to obtain the new rotation from
% body to world, we need to postmultiply it:
% * let W be world frame,
% * let OB be old body frame
% * let NB be new body frame
% q^W_NB = q^W_OB * q^OB_NB
% q^W_OB = qHat
% q^OB_NB = deltaPhiToDeltaQ(omegaHat * dt)
qHat = quaternionProduct(qHat, deltaPhiToDeltaQ(omegaHat * dt)); % + omegad * dt^2 / 2)); 
omegaHat = omegaHat + omegad * dt;

% a priori covariance
P = F * P * F' + Q;

% measurement residual
% * let W be world frame,
% * let OB be old body frame
% * let NB be new body frame
% * let M be measured frame
% q^W_OB = qHat
% in the end, we want deltaQ = deltaPhiToDeltaQ(deltaPhi) = q^OB_NB
% with deltaPhi = KPhi * deltaQToDeltaPhi(q^OB_M)
% hence, the error term yQ needs to be a correction in body frame: 
% yQ = q^OB_M
% q^OB_M = q^OB_W * q^W_M = (q^W_OB)^-1 * zQ
yQ = quaternionProduct(quaternionInverse(qHat), zQ);
yPhi = deltaQToDeltaPhi(yQ);
yOmega = zOmega - (omegaHat + bHat);
y = [yPhi; yOmega];

% Kalman gain matrix
S = H * P * H' + R;
K = P * H' * inv(S);

% a posteriori state
deltaX = K * y;
deltaPhi = deltaX(1 : 3);
deltaQ = deltaPhiToDeltaQ(deltaPhi);
deltaOmega = deltaX(4 : 6);
deltaB = deltaX(7 : 9);

% deltaQ is expressed in body frame, so to obtain the new rotation from
% body to world, we need to postmultiply it:
% * let W be world frame,
% * let OB be old body frame
% * let NB be new body frame
% q^W_NB = q^W_OB * q^OB_NB
% with
% q^W_OB = qHat
% q^OB_NB = deltaQ
% so: qHat <- qHat * deltaQ
qHat = quaternionProduct(qHat, deltaQ);
qHat = normalizeQuaternion(qHat);
omegaHat = omegaHat + deltaOmega;
bHat = bHat + deltaB;

% a posteriori covariance
P = (eye(size(P, 1)) - K * H) * P;

% pack
state.qHat = qHat;
state.omegaHat = omegaHat;
state.bHat = bHat;
state.P = P;

end



function [A, B] = createContinuousTimeSystem(omegaHat)
% A = [-tilde(omegaHat), eye(3), zeros(3, 3);
%      zeros(3, 9);
%      zeros(3, 9)];

A = [-1/2 * tilde(omegaHat), eye(3), zeros(3, 3);
     zeros(3, 9);
     zeros(3, 9)];
B = [zeros(3, 3);
     eye(3);
     zeros(3, 3)];
% B = zeros(9, 0);
end

% copied from kalmd
% originates from Charles F. Van Loan - Computing Integrals Involving the
% Matrix Exponential
function [ad, bd, Qd, Rd] = discretize(a, b, g, qn, rn, Ts)

Nx = size(a,1);

% Check symmetry and dimensions of Qn,Rn
if norm(qn'-qn,1) > 100*eps*norm(qn,1),
   warning('Qn is not symmetric and has been replaced by (Q+Q'')/2).')
end
if norm(rn'-rn,1) > 100*eps*norm(rn,1),
   warning('Rn is not symmetric and has been replaced by (R+R'')/2).')
end

% Form G*Q*G', enforce symmetry and check positivity
qn = g * qn *g';
qn = (qn+qn')/2;
rn = (rn+rn')/2;
if min(real(eig(rn)))<=0,
   error('The covariance matrix Rn must be positive definite.')
end

% Discretize the state-space system.
[ad,bd] = c2d(a,b,Ts);

% Compute discrete equivalent of continuous noise 
M = [-a  qn ; zeros(Nx) a'];
phi = expm(M*Ts);
phi12 = phi(1:Nx,Nx+1:2*Nx);
phi22 = phi(Nx+1:2*Nx,Nx+1:2*Nx);
Qd = phi22'*phi12;
Qd = (Qd+Qd')/2;        % Make sure Qd is symmetric
Rd = rn/Ts;
end

% note: G not used.
function [Ad, Bd, Qd, Rd] = discretize2(A, B, G, Q, R, deltaT)
n = size(A, 1);
m = size(B, 2);
C = [-A', Q, zeros(n, m);
     zeros(n, n), A, B;
     zeros(m, n), zeros(m, n), zeros(m, m)];

% expC = expm(C * deltaT);
nApprox = 3;
expC = expmApprox(C * deltaT, nApprox);
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
