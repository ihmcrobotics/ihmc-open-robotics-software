package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class FloatingBaseRigidBodyDynamicsCalculator
{
   private static final int large = 1000;
   private static final double tolerance = 0.0001;

   private static final LinearSolver<DenseMatrix64F> pseudoInverseSolver = LinearSolverFactory.pseudoInverse(true);

   private final DenseMatrix64F matrixTranspose = new DenseMatrix64F(large, large);
   private final DenseMatrix64F localVector = new DenseMatrix64F(large, 1);

   public FloatingBaseRigidBodyDynamicsCalculator()
   {}


   public void computeQddotGivenRho(DenseMatrix64F floatingBaseMassMatrix, DenseMatrix64F floatingBaseCoriolisMatrix,
         DenseMatrix64F floatingBaseContactForceJacobianMatrix, DenseMatrix64F qddotToPack, DenseMatrix64F rho)
   {
      computeJacobianTranspose(floatingBaseContactForceJacobianMatrix, matrixTranspose);

      CommonOps.scale(-1.0, floatingBaseCoriolisMatrix);
      CommonOps.multAdd(matrixTranspose, rho, floatingBaseCoriolisMatrix);

      pseudoInverseSolver.setA(floatingBaseMassMatrix);
      pseudoInverseSolver.solve(floatingBaseCoriolisMatrix, qddotToPack);
   }

   public void computeRhoGivenQddot(DenseMatrix64F floatingBaseMassMatrix, DenseMatrix64F floatingBaseCoriolisMatrix,
         DenseMatrix64F floatingBaseContactForceJacobianMatrix, DenseMatrix64F qddot, DenseMatrix64F rhoToPack)
   {
      computeJacobianTranspose(floatingBaseContactForceJacobianMatrix, matrixTranspose);

      CommonOps.multAdd(floatingBaseMassMatrix, qddot, floatingBaseCoriolisMatrix);

      pseudoInverseSolver.setA(matrixTranspose);
      pseudoInverseSolver.solve(floatingBaseCoriolisMatrix, rhoToPack);
   }

   public void computeTauGivenRhoAndQddot(DenseMatrix64F bodyMassMatrix, DenseMatrix64F bodyCoriolisMatrix, DenseMatrix64F bodyContactForceJacobianMatrix,
         DenseMatrix64F qddot, DenseMatrix64F rho, DenseMatrix64F tauToPack)
   {
      computeJacobianTranspose(bodyContactForceJacobianMatrix, matrixTranspose);

      CommonOps.mult(bodyMassMatrix, qddot, tauToPack);
      CommonOps.multAdd(-1.0, matrixTranspose, rho, tauToPack);
      CommonOps.addEquals(tauToPack, bodyCoriolisMatrix);
   }

   public void computeTauGivenRho(DenseMatrix64F floatingBaseMassMatrix, DenseMatrix64F floatingBaseCoriolisMatrix, DenseMatrix64F floatingBaseContactForceJacobianMatrix,
         DenseMatrix64F bodyMassMatrix, DenseMatrix64F bodyCoriolisMatrix, DenseMatrix64F bodyContactForceJacobianMatrix, DenseMatrix64F rho, DenseMatrix64F tauToPack)
   {
      computeJacobianTranspose(bodyContactForceJacobianMatrix, matrixTranspose);

      localVector.reshape(bodyMassMatrix.getNumCols(), 1);
      computeQddotGivenRho(floatingBaseMassMatrix, floatingBaseCoriolisMatrix, floatingBaseContactForceJacobianMatrix, localVector, rho);
      computeTauGivenRhoAndQddot(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobianMatrix, localVector, rho, tauToPack);
   }

   public void computeTauGivenQddot(DenseMatrix64F floatingBaseMassMatrix, DenseMatrix64F floatingBaseCoriolisMatrix, DenseMatrix64F floatingBaseContactForceJacobianMatrix,
         DenseMatrix64F bodyMassMatrix, DenseMatrix64F bodyCoriolisMatrix, DenseMatrix64F bodyContactForceJacobianMatrix, DenseMatrix64F qddot, DenseMatrix64F tauToPack)
   {
      computeJacobianTranspose(bodyContactForceJacobianMatrix, matrixTranspose);

      localVector.reshape(bodyContactForceJacobianMatrix.getNumRows(), 1);
      computeRhoGivenQddot(floatingBaseMassMatrix, floatingBaseCoriolisMatrix, floatingBaseContactForceJacobianMatrix, qddot, localVector);
      computeTauGivenRhoAndQddot(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobianMatrix, qddot, localVector, tauToPack);
   }

   public boolean areFloatingBaseRigidBodyDynamicsSatisfied(DenseMatrix64F floatingBaseMassMatrix, DenseMatrix64F floatingBaseCoriolisMatrix,
         DenseMatrix64F floatingBaseContactForceJacobianMatrix, DenseMatrix64F bodyMassMatrix, DenseMatrix64F bodyCoriolisMatrix,
         DenseMatrix64F bodyContactForceJacobianMatrix, DenseMatrix64F qddot, DenseMatrix64F tau, DenseMatrix64F rho)
   {
      if (!areFloatingBaseDynamicsSatisfied(floatingBaseMassMatrix, floatingBaseCoriolisMatrix, floatingBaseContactForceJacobianMatrix, qddot, rho))
         return false;

      return areRigidBodyDynamicsSatisfied(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobianMatrix, qddot, tau, rho);
   }

   public boolean areFloatingBaseDynamicsSatisfied(DenseMatrix64F floatingBaseMassMatrix, DenseMatrix64F floatingBaseCoriolisMatrix,
         DenseMatrix64F floatingBaseContactForceJacobianMatrix, DenseMatrix64F qddot, DenseMatrix64F rho)
   {
      computeJacobianTranspose(floatingBaseContactForceJacobianMatrix, matrixTranspose);

      CommonOps.multAdd(floatingBaseMassMatrix, qddot, floatingBaseCoriolisMatrix);
      CommonOps.multAdd(-1.0, matrixTranspose, rho, floatingBaseCoriolisMatrix);

      return equalsZero(floatingBaseCoriolisMatrix, tolerance);
   }

   public boolean areRigidBodyDynamicsSatisfied(DenseMatrix64F bodyMassMatrix, DenseMatrix64F bodyCoriolisMatrix, DenseMatrix64F bodyContactForceJacobianMatrix,
         DenseMatrix64F qddot, DenseMatrix64F tau, DenseMatrix64F rho)
   {
      computeJacobianTranspose(bodyContactForceJacobianMatrix, matrixTranspose);

      CommonOps.multAdd(bodyMassMatrix, qddot, bodyCoriolisMatrix);
      CommonOps.multAdd(-1.0, matrixTranspose, rho, bodyCoriolisMatrix);
      CommonOps.subtractEquals(bodyCoriolisMatrix, tau);

      return equalsZero(bodyCoriolisMatrix, tolerance);
   }

   private void computeJacobianTranspose(DenseMatrix64F jacobian, DenseMatrix64F jacobianTransposeToPack)
   {
      jacobianTransposeToPack.reshape(jacobian.getNumCols(), jacobian.getNumRows());
      jacobianTransposeToPack.zero();
      CommonOps.transpose(jacobian, jacobianTransposeToPack);
   }

   private static boolean equalsZero(DenseMatrix64F matrix, double tolerance)
   {
      for (int rowIndex = 0; rowIndex < matrix.getNumRows(); rowIndex++)
      {
         for (int colIndex = 0; colIndex < matrix.getNumCols(); colIndex++)
         {
            if (!equals(matrix.get(rowIndex, colIndex), 0.0, tolerance))
               return false;
         }
      }

      return true;
   }

   private static boolean equals(double a, double b, double tolerance)
   {
      if (Math.abs(a - b) > tolerance)
         return false;

      return true;
   }
}
