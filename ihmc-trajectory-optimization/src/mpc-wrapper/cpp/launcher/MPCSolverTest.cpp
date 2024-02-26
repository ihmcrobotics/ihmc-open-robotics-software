#include "hpipm-cpp/hpipm-cpp.hpp"

#include <chrono>
#include <iostream>
#include <vector>

#include "Eigen/Core"


int main() {
  // setup QP
  const unsigned int N = 20;

  std::vector<hpipm::OcpQp> qp(N+1);
  // dynamics
  Eigen::MatrixXd A(12, 12), B(12, 4);
  A << 1.,      0.,     0., 0., 0., 0., 0.1,     0.,     0.,  0.,     0.,     0.    ,
       0.,      1.,     0., 0., 0., 0., 0.,      0.1,    0.,  0.,     0.,     0.    ,
       0.,      0.,     1., 0., 0., 0., 0.,      0.,     0.1, 0.,     0.,     0.    ,
       0.0488,  0.,     0., 1., 0., 0., 0.0016,  0.,     0.,  0.0992, 0.,     0.    ,
       0.,     -0.0488, 0., 0., 1., 0., 0.,     -0.0016, 0.,  0.,     0.0992, 0.    ,
       0.,      0.,     0., 0., 0., 1., 0.,      0.,     0.,  0.,     0.,     0.0992,
       0.,      0.,     0., 0., 0., 0., 1.,      0.,     0.,  0.,     0.,     0.    ,
       0.,      0.,     0., 0., 0., 0., 0.,      1.,     0.,  0.,     0.,     0.    ,
       0.,      0.,     0., 0., 0., 0., 0.,      0.,     1.,  0.,     0.,     0.    ,
       0.9734,  0.,     0., 0., 0., 0., 0.0488,  0.,     0.,  0.9846, 0.,     0.    ,
       0.,     -0.9734, 0., 0., 0., 0., 0.,     -0.0488, 0.,  0.,     0.9846, 0.    ,
       0.,      0.,     0., 0., 0., 0., 0.,      0.,     0.,  0.,     0.,     0.9846;
  B << 0.,      -0.0726,  0.,     0.0726,
      -0.0726,  0.,      0.0726, 0.    ,
      -0.0152,  0.0152, -0.0152, 0.0152,
      -0.,     -0.0006, -0.,     0.0006,
       0.0006,   0.,     -0.0006, 0.0000,
       0.0106,   0.0106,  0.0106, 0.0106,
       0,       -1.4512,  0.,     1.4512,
      -1.4512,  0.,      1.4512, 0.    ,
      -0.3049,  0.3049, -0.3049, 0.3049,
      -0.,     -0.0236,  0.,     0.0236,
       0.0236,   0.,     -0.0236, 0.    ,
       0.2107,   0.2107,  0.2107, 0.2107;
  const Eigen::VectorXd b = Eigen::VectorXd::Zero(12);
  for (uint32_t i=0; i<N; ++i) {
    qp[i].A = A;
    qp[i].B = B;
    qp[i].b = b;
  }
  // cost
  Eigen::MatrixXd Q(12, 12), S(4, 12), R(4, 4);
  Q.setZero(); Q.diagonal() << 0, 0, 10., 10., 10., 10., 0, 0, 0, 5., 5., 5.;
  S.setZero();
  R.setZero(); R.diagonal() << 0.1, 0.1, 0.1, 0.1;
  Eigen::VectorXd x_ref(12);
  x_ref <<  0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  const Eigen::VectorXd q = - Q * x_ref;
  const Eigen::VectorXd r = Eigen::VectorXd::Zero(4);
  for (uint32_t i=0; i<N; ++i) {
    qp[i].Q = Q;
    qp[i].R = R;
    qp[i].S = S;
    qp[i].q = q;
    qp[i].r = r;
  }
  qp[N].Q = Q;
  qp[N].q = q;
  // constraints
  const bool use_mask_for_one_sided_constraints = true;
  for (uint32_t i=1; i<=N; ++i) {
    constexpr double soft_inf = 1.0e10;
    qp[i].idxbx = {0, 1, 5};
    qp[i].lbx = (Eigen::VectorXd(3) << -M_PI/6.0, -M_PI/6.0, -1.0).finished();
    qp[i].ubx = (Eigen::VectorXd(3) << M_PI/6.0, M_PI/6.0, soft_inf).finished();
    if (use_mask_for_one_sided_constraints) {
      qp[i].ubx_mask = (Eigen::VectorXd(3) << 1.0, 1.0, 0.0).finished(); // this mask disables upper bound by ubx[2]
    }
  }
  for (uint32_t i=0; i<N; ++i) {
    constexpr double u0 = 10.5916;
    qp[i].idxbu = {0, 1, 2, 3};
    qp[i].lbu = (Eigen::VectorXd(4) << 9.6-u0, 9.6-u0, 9.6-u0, 9.6-u0).finished();
    qp[i].ubu = (Eigen::VectorXd(4) << 13.0-u0, 13.0-u0, 13.0-u0, 13.0-u0).finished();
  }

  hpipm::OcpQpIpmSolverSettings solver_settings;
  solver_settings.mode = hpipm::HpipmMode::Balance;
  solver_settings.iter_max = 30;
  solver_settings.alpha_min = 1e-8;
  solver_settings.mu0 = 1e2;
  solver_settings.tol_stat = 1e-04;
  solver_settings.tol_eq = 1e-04;
  solver_settings.tol_ineq = 1e-04;
  solver_settings.tol_comp = 1e-04;
  solver_settings.reg_prim = 1e-12;
  solver_settings.warm_start = 1;
  solver_settings.pred_corr = 1;
  solver_settings.ric_alg = 0;
  solver_settings.split_step = 1;

  std::vector<hpipm::OcpQpSolution> solution(N+1);
  hpipm::OcpQpIpmSolver solver(qp, solver_settings);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(12);
  for (uint32_t i=0; i<N; ++i) {
    constexpr double u0 = 10.5916;
    solution[i].x = x;
    solution[i].u.resize(4);
    solution[i].u.fill(u0);
  }
  solution[N].x = x;

  Eigen::VectorXd x0 = x;

  const uint32_t sim_steps = 50;
  for (uint32_t t=0; t<sim_steps; ++t) {
    std::cout << "t: " << t << ", x: " << x.transpose() << std::endl;
    x0 = x;
    if (solver.solve(x0, qp, solution) != hpipm::HpipmStatus::Success) return 1;
    const auto u0 = solution[0].u;
    x = A * x + B * u0 + b;
  }

  std::cout << "t: " << sim_steps << ", x: " << x.transpose() << std::endl;
  return 0;
}