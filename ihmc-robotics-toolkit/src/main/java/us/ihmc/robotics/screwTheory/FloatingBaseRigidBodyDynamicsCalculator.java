package us.ihmc.robotics.screwTheory;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

public class FloatingBaseRigidBodyDynamicsCalculator
{
   private static final int large = 1000;
   private static final double tolerance = 0.0001;

   private static final LinearSolverDense<DMatrixRMaj> pseudoInverseSolver = LinearSolverFactory_DDRM.pseudoInverse(true);

   private final DMatrixRMaj matrixTranspose = new DMatrixRMaj(large, large);
   private final DMatrixRMaj localVector = new DMatrixRMaj(large, 1);

   public FloatingBaseRigidBodyDynamicsCalculator()
   {}


   public void computeQddotGivenRho(DMatrixRMaj floatingBaseMassMatrix, DMatrixRMaj floatingBaseCoriolisMatrix,
         DMatrixRMaj floatingBaseContactForceJacobianMatrix, DMatrixRMaj qddotToPack, DMatrixRMaj rho)
   {
      computeJacobianTranspose(floatingBaseContactForceJacobianMatrix, matrixTranspose);

      CommonOps_DDRM.scale(-1.0, floatingBaseCoriolisMatrix);
      CommonOps_DDRM.multAdd(matrixTranspose, rho, floatingBaseCoriolisMatrix);

      pseudoInverseSolver.setA(floatingBaseMassMatrix);
      pseudoInverseSolver.solve(floatingBaseCoriolisMatrix, qddotToPack);
   }

   public void computeRhoGivenQddot(DMatrixRMaj floatingBaseMassMatrix, DMatrixRMaj floatingBaseCoriolisMatrix,
         DMatrixRMaj floatingBaseContactForceJacobianMatrix, DMatrixRMaj qddot, DMatrixRMaj rhoToPack)
   {
      computeJacobianTranspose(floatingBaseContactForceJacobianMatrix, matrixTranspose);

      CommonOps_DDRM.multAdd(floatingBaseMassMatrix, qddot, floatingBaseCoriolisMatrix);

      pseudoInverseSolver.setA(matrixTranspose);
      pseudoInverseSolver.solve(floatingBaseCoriolisMatrix, rhoToPack);
   }

   public void computeTauGivenRhoAndQddot(DMatrixRMaj bodyMassMatrix, DMatrixRMaj bodyCoriolisMatrix, DMatrixRMaj bodyContactForceJacobianMatrix,
         DMatrixRMaj qddot, DMatrixRMaj rho, DMatrixRMaj tauToPack)
   {
      computeJacobianTranspose(bodyContactForceJacobianMatrix, matrixTranspose);

      CommonOps_DDRM.mult(bodyMassMatrix, qddot, tauToPack);
      CommonOps_DDRM.multAdd(-1.0, matrixTranspose, rho, tauToPack);
      CommonOps_DDRM.addEquals(tauToPack, bodyCoriolisMatrix);
   }

   public void computeTauGivenRho(DMatrixRMaj floatingBaseMassMatrix, DMatrixRMaj floatingBaseCoriolisMatrix, DMatrixRMaj floatingBaseContactForceJacobianMatrix,
         DMatrixRMaj bodyMassMatrix, DMatrixRMaj bodyCoriolisMatrix, DMatrixRMaj bodyContactForceJacobianMatrix, DMatrixRMaj rho, DMatrixRMaj tauToPack)
   {
      computeJacobianTranspose(bodyContactForceJacobianMatrix, matrixTranspose);

      localVector.reshape(bodyMassMatrix.getNumCols(), 1);
      computeQddotGivenRho(floatingBaseMassMatrix, floatingBaseCoriolisMatrix, floatingBaseContactForceJacobianMatrix, localVector, rho);
      computeTauGivenRhoAndQddot(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobianMatrix, localVector, rho, tauToPack);
   }

   public void computeTauGivenQddot(DMatrixRMaj floatingBaseMassMatrix, DMatrixRMaj floatingBaseCoriolisMatrix, DMatrixRMaj floatingBaseContactForceJacobianMatrix,
         DMatrixRMaj bodyMassMatrix, DMatrixRMaj bodyCoriolisMatrix, DMatrixRMaj bodyContactForceJacobianMatrix, DMatrixRMaj qddot, DMatrixRMaj tauToPack)
   {
      computeJacobianTranspose(bodyContactForceJacobianMatrix, matrixTranspose);

      localVector.reshape(bodyContactForceJacobianMatrix.getNumRows(), 1);
      computeRhoGivenQddot(floatingBaseMassMatrix, floatingBaseCoriolisMatrix, floatingBaseContactForceJacobianMatrix, qddot, localVector);
      computeTauGivenRhoAndQddot(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobianMatrix, qddot, localVector, tauToPack);
   }

   public boolean areFloatingBaseRigidBodyDynamicsSatisfied(DMatrixRMaj floatingBaseMassMatrix, DMatrixRMaj floatingBaseCoriolisMatrix,
         DMatrixRMaj floatingBaseContactForceJacobianMatrix, DMatrixRMaj bodyMassMatrix, DMatrixRMaj bodyCoriolisMatrix,
         DMatrixRMaj bodyContactForceJacobianMatrix, DMatrixRMaj qddot, DMatrixRMaj tau, DMatrixRMaj rho)
   {
      if (!areFloatingBaseDynamicsSatisfied(floatingBaseMassMatrix, floatingBaseCoriolisMatrix, floatingBaseContactForceJacobianMatrix, qddot, rho))
         return false;

      return areRigidBodyDynamicsSatisfied(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobianMatrix, qddot, tau, rho);
   }

   public boolean areFloatingBaseDynamicsSatisfied(DMatrixRMaj floatingBaseMassMatrix, DMatrixRMaj floatingBaseCoriolisMatrix,
         DMatrixRMaj floatingBaseContactForceJacobianMatrix, DMatrixRMaj qddot, DMatrixRMaj rho)
   {
      computeJacobianTranspose(floatingBaseContactForceJacobianMatrix, matrixTranspose);

      CommonOps_DDRM.multAdd(floatingBaseMassMatrix, qddot, floatingBaseCoriolisMatrix);
      CommonOps_DDRM.multAdd(-1.0, matrixTranspose, rho, floatingBaseCoriolisMatrix);

      return equalsZero(floatingBaseCoriolisMatrix, tolerance);
   }

   public boolean areRigidBodyDynamicsSatisfied(DMatrixRMaj bodyMassMatrix, DMatrixRMaj bodyCoriolisMatrix, DMatrixRMaj bodyContactForceJacobianMatrix,
         DMatrixRMaj qddot, DMatrixRMaj tau, DMatrixRMaj rho)
   {
      computeJacobianTranspose(bodyContactForceJacobianMatrix, matrixTranspose);

      CommonOps_DDRM.multAdd(bodyMassMatrix, qddot, bodyCoriolisMatrix);
      CommonOps_DDRM.multAdd(-1.0, matrixTranspose, rho, bodyCoriolisMatrix);
      CommonOps_DDRM.subtractEquals(bodyCoriolisMatrix, tau);

      return equalsZero(bodyCoriolisMatrix, tolerance);
   }

   private void computeJacobianTranspose(DMatrixRMaj jacobian, DMatrixRMaj jacobianTransposeToPack)
   {
      jacobianTransposeToPack.reshape(jacobian.getNumCols(), jacobian.getNumRows());
      jacobianTransposeToPack.zero();
      CommonOps_DDRM.transpose(jacobian, jacobianTransposeToPack);
   }

   private static boolean equalsZero(DMatrixRMaj matrix, double tolerance)
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
