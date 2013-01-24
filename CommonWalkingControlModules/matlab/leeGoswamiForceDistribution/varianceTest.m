n = 3;
epsilon = 1e-5;
G = eye(n) - 1/n * ones(n, n);
cvx_begin
  variable rho(n)
%   minimize(sum(square(rho - sum(rho) / n)) + epsilon * rho' * rho);
%   minimize(sum(square(rho - 1/n *sum(rho))) + epsilon*quad_form(rho, eye(n)));
  minimize(sum(square(G * rho)) + epsilon*quad_form(rho, eye(n)));
  subject to
  rho >= 1
  cvx_problem
cvx_end