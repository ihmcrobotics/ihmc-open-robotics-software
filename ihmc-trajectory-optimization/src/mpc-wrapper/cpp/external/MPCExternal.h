
#include "hpipm-cpp/hpipm-cpp.hpp"

#include <chrono>
#include <iostream>
#include <vector>

#include "Eigen/Core"


class MPCExternal
{
   public:
      MPCExternal() {}

      void setMatrix_A(double* array, uint32_t rows, uint32_t cols, uint32_t index);

      void setMatrix_B(double* array, uint32_t rows, uint32_t cols, uint32_t index);

      void setMatrix_Q(double* array, uint32_t rows, uint32_t cols, uint32_t index);

      void setMatrix_S(double* array, uint32_t rows, uint32_t cols, uint32_t index);

      void setMatrix_R(double* array, uint32_t rows, uint32_t cols, uint32_t index);

      void setVector_b(double* array, uint32_t rows, uint32_t index);

      void setVector_q(double* array, uint32_t rows, uint32_t index);

      void setVector_r(double* array, uint32_t rows, uint32_t index);

      void solveQP();

      void setQPSolverSettings();

   private:

      const unsigned int N = 20;

      hpipm::OcpQpIpmSolverSettings solver_settings;

      std::vector<hpipm::OcpQp> qp;

      Eigen::MatrixXd _A, _B;

      Eigen::VectorXd _b;

};