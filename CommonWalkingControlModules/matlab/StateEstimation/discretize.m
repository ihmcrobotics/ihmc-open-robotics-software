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