package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

public class FloatingBaseRigidBodyDynamicsTools
{
   private static final int large = 1000;
   private static final double tolerance = 0.0001;

   private static final LinearSolver<DenseMatrix64F> pseudoInverseSolver = LinearSolverFactory.pseudoInverse(true);

   private static final DenseMatrix64F matrixTranspose = new DenseMatrix64F(large, large);
   private static final DenseMatrix64F localVector = new DenseMatrix64F(large, 1);

   public static void computeQddotGivenRho(DenseMatrix64F floatingBaseMassMatrix, DenseMatrix64F floatingBaseCoriolisMatrix,
         DenseMatrix64F floatingBaseContactForceJacobianMatrix, DenseMatrix64F qddotToPack, DenseMatrix64F rho)
   {
      computeJacobianTranspose(floatingBaseContactForceJacobianMatrix, matrixTranspose);

      CommonOps.multAdd(matrixTranspose, rho, floatingBaseCoriolisMatrix);

      pseudoInverseSolver.setA(floatingBaseMassMatrix);
      pseudoInverseSolver.solve(floatingBaseCoriolisMatrix, qddotToPack);
   }

   public static void computeRhoGivenQddot(DenseMatrix64F floatingBaseMassMatrix, DenseMatrix64F floatingBaseCoriolisMatrix,
         DenseMatrix64F floatingBaseContactForceJacobianMatrix, DenseMatrix64F qddot, DenseMatrix64F rhoToPack)
   {
      computeJacobianTranspose(floatingBaseContactForceJacobianMatrix, matrixTranspose);

      CommonOps.multAdd(floatingBaseMassMatrix, qddot, floatingBaseCoriolisMatrix);

      pseudoInverseSolver.setA(matrixTranspose);
      pseudoInverseSolver.solve(floatingBaseCoriolisMatrix, rhoToPack);
   }

   public static void computeTauGivenRhoAndQddot(DenseMatrix64F bodyMassMatrix, DenseMatrix64F bodyCoriolisMatrix, DenseMatrix64F bodyContactForceJacobianMatrix,
         DenseMatrix64F qddot, DenseMatrix64F rho, DenseMatrix64F tauToPack)
   {
      computeJacobianTranspose(bodyContactForceJacobianMatrix, matrixTranspose);

      CommonOps.mult(bodyMassMatrix, qddot, tauToPack);
      CommonOps.multAdd(-1.0, matrixTranspose, rho, tauToPack);
      CommonOps.addEquals(tauToPack, bodyCoriolisMatrix);
   }

   public static void computeTauGivenRho(DenseMatrix64F floatingBaseMassMatrix, DenseMatrix64F floatingBaseCoriolisMatrix, DenseMatrix64F floatingBaseContactForceJacobianMatrix,
         DenseMatrix64F bodyMassMatrix, DenseMatrix64F bodyCoriolisMatrix, DenseMatrix64F bodyContactForceJacobianMatrix, DenseMatrix64F rho, DenseMatrix64F tauToPack)
   {
      computeJacobianTranspose(bodyContactForceJacobianMatrix, matrixTranspose);

      localVector.reshape(bodyMassMatrix.getNumCols(), 1);
      FloatingBaseRigidBodyDynamicsTools.computeQddotGivenRho(floatingBaseMassMatrix, floatingBaseCoriolisMatrix, floatingBaseContactForceJacobianMatrix, localVector, rho);
      FloatingBaseRigidBodyDynamicsTools.computeTauGivenRhoAndQddot(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobianMatrix, localVector, rho, tauToPack);
   }

   public static void computeTauGivenQddot(DenseMatrix64F floatingBaseMassMatrix, DenseMatrix64F floatingBaseCoriolisMatrix, DenseMatrix64F floatingBaseContactForceJacobianMatrix,
         DenseMatrix64F bodyMassMatrix, DenseMatrix64F bodyCoriolisMatrix, DenseMatrix64F bodyContactForceJacobianMatrix, DenseMatrix64F qddot, DenseMatrix64F tauToPack)
   {
      computeJacobianTranspose(bodyContactForceJacobianMatrix, matrixTranspose);

      localVector.reshape(bodyMassMatrix.getNumCols(), 1);
      FloatingBaseRigidBodyDynamicsTools.computeRhoGivenQddot(floatingBaseMassMatrix, floatingBaseCoriolisMatrix, floatingBaseContactForceJacobianMatrix, qddot, localVector);
      FloatingBaseRigidBodyDynamicsTools.computeTauGivenRhoAndQddot(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobianMatrix, qddot, localVector, tauToPack);
   }

   public static boolean areFloatingBaseRigidBodyDynamicsSatisfied(DenseMatrix64F floatingBaseMassMatrix, DenseMatrix64F floatingBaseCoriolisMatrix,
         DenseMatrix64F floatingBaseContactForceJacobianMatrix, DenseMatrix64F bodyMassMatrix, DenseMatrix64F bodyCoriolisMatrix,
         DenseMatrix64F bodyContactForceJacobianMatrix, DenseMatrix64F qddot, DenseMatrix64F tau, DenseMatrix64F rho)
   {
      if (!areFloatingBaseDynamicsSatisfied(floatingBaseMassMatrix, floatingBaseCoriolisMatrix, floatingBaseContactForceJacobianMatrix, qddot, rho))
         return false;

      return areRigidBodyDynamicsSatisfied(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobianMatrix, qddot, tau, rho);
   }

   public static boolean areFloatingBaseDynamicsSatisfied(DenseMatrix64F floatingBaseMassMatrix, DenseMatrix64F floatingBaseCoriolisMatrix,
         DenseMatrix64F floatingBaseContactForceJacobianMatrix, DenseMatrix64F qddot, DenseMatrix64F rho)
   {
      computeJacobianTranspose(floatingBaseContactForceJacobianMatrix, matrixTranspose);

      CommonOps.multAdd(floatingBaseMassMatrix, qddot, floatingBaseCoriolisMatrix);
      CommonOps.multAdd(-1.0, matrixTranspose, rho, floatingBaseCoriolisMatrix);

      return equalsZero(floatingBaseCoriolisMatrix, tolerance);
   }

   public static boolean areRigidBodyDynamicsSatisfied(DenseMatrix64F bodyMassMatrix, DenseMatrix64F bodyCoriolisMatrix, DenseMatrix64F bodyContactForceJacobianMatrix,
         DenseMatrix64F qddot, DenseMatrix64F tau, DenseMatrix64F rho)
   {
      computeJacobianTranspose(bodyContactForceJacobianMatrix, matrixTranspose);

      CommonOps.multAdd(bodyMassMatrix, qddot, bodyCoriolisMatrix);
      CommonOps.multAdd(-1.0, matrixTranspose, rho, bodyCoriolisMatrix);
      CommonOps.subtractEquals(bodyCoriolisMatrix, tau);

      return equalsZero(bodyCoriolisMatrix, tolerance);
   }

   private static void computeJacobianTranspose(DenseMatrix64F jacobian, DenseMatrix64F jacobianTransposeToPack)
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
