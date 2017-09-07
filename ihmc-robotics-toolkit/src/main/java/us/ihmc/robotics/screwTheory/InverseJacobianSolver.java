package us.ihmc.robotics.screwTheory;

import org.ejml.alg.dense.linsol.LinearSolverSafe;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;

public class InverseJacobianSolver implements JacobianSolver
{
   private final LinearSolver<DenseMatrix64F> solver;
   private final DenseMatrix64F jacobianMatrix;
   
   public InverseJacobianSolver(int matrixSize)
   {
      solver = new LinearSolverSafe<DenseMatrix64F>(LinearSolverFactory.leastSquaresQrPivot(true, false));
      jacobianMatrix = new DenseMatrix64F(matrixSize, matrixSize);
   }

   @Override
   public void solve(DenseMatrix64F solutionToPack, DenseMatrix64F vector)
   {
      if (!solver.setA(jacobianMatrix))
         throw new RuntimeException("jacobian is singular");
      solver.solve(vector, solutionToPack);
   }

   @Override
   public void setJacobian(DenseMatrix64F jacobianMatrix)
   {
      this.jacobianMatrix.set(jacobianMatrix);      
   }
}
