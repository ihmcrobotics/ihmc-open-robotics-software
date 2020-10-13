package us.ihmc.robotics.screwTheory;

import org.ejml.LinearSolverSafe;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

public class InverseJacobianSolver implements JacobianSolver
{
   private final LinearSolverDense<DMatrixRMaj> solver;
   private final DMatrixRMaj jacobianMatrix;
   
   public InverseJacobianSolver(int matrixSize)
   {
      solver = new LinearSolverSafe<DMatrixRMaj>(LinearSolverFactory_DDRM.leastSquaresQrPivot(true, false));
      jacobianMatrix = new DMatrixRMaj(matrixSize, matrixSize);
   }

   @Override
   public void solve(DMatrixRMaj solutionToPack, DMatrixRMaj vector)
   {
      if (!solver.setA(jacobianMatrix))
         throw new RuntimeException("jacobian is singular");
      solver.solve(vector, solutionToPack);
   }

   @Override
   public void setJacobian(DMatrixRMaj jacobianMatrix)
   {
      this.jacobianMatrix.set(jacobianMatrix);      
   }
}
