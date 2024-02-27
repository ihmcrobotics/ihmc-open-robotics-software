#include "MPCExternal.h"

void MPCExternal::setMatrix_A(double* array, uint32_t rows, uint32_t cols, uint32_t index) 
{
    Eigen::Map<Eigen::MatrixXd> A(array, rows, cols);
    qp[index].A = A;
    _A = A;
}

void MPCExternal::setMatrix_B(double* array, uint32_t rows, uint32_t cols, uint32_t index) 
{
    Eigen::Map<Eigen::MatrixXd> B(array, rows, cols);
    qp[index].B = B;
    _B = B;
}


void MPCExternal::setMatrix_Q(double* array, uint32_t rows, uint32_t cols, uint32_t index) 
{
    Eigen::Map<Eigen::MatrixXd> Q(array, rows, cols);
    qp[index].Q = Q;
}

void MPCExternal::setMatrix_S(double* array, uint32_t rows, uint32_t cols, uint32_t index) 
{
    Eigen::Map<Eigen::MatrixXd> S(array, rows, cols);
    qp[index].S = S;
}

void MPCExternal::setMatrix_R(double* array, uint32_t rows, uint32_t cols, uint32_t index) 
{
    Eigen::Map<Eigen::MatrixXd> R(array, rows, cols);
    qp[index].R = R;
}

void MPCExternal::setVector_b(double* array, uint32_t size, uint32_t index) 
{
    Eigen::Map<Eigen::VectorXd> b(array, size);
    qp[index].b = b;
    _b = b;
}

void MPCExternal::setVector_q(double* array, uint32_t size, uint32_t index) 
{
    Eigen::Map<Eigen::VectorXd> q(array, size);
    qp[index].q = q;
}

void MPCExternal::setVector_r(double* array, uint32_t size, uint32_t index) 
{
    Eigen::Map<Eigen::VectorXd> r(array, size);
    qp[index].r = r;
}

void MPCExternal::setQPSolverSettings()
{
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
}

void MPCExternal::solveQP() 
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
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
        
        if (solver.solve(x0, qp, solution) != hpipm::HpipmStatus::Success) 
            return;
        
        const auto u0 = solution[0].u;
        x = _A * x + _B * u0 + _b;
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
}
