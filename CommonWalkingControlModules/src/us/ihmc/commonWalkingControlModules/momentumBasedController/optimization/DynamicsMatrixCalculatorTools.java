package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

public class DynamicsMatrixCalculatorTools
{
   private static final int large = 1000;
   private static final double tolerance = 0.5;

   private static final LinearSolver<DenseMatrix64F> pseudoInverseSolver = LinearSolverFactory.pseudoInverse(true);

   private static final DenseMatrix64F matrixTranspose = new DenseMatrix64F(large, large);
   private static final DenseMatrix64F localMassMatrix = new DenseMatrix64F(large, large);
   private static final DenseMatrix64F localCoriolisMatrix = new DenseMatrix64F(large, large);
   private static final DenseMatrix64F localContactJacobian = new DenseMatrix64F(large, large);

   /**
    * <p>
    * Computes the required contact forces given the joint accelerations using the rigid-body dynamics for the floating body.
    * </p>
    *
    * @param dynamicsMatrixCalculator
    * @param qddot
    * @param rhoToPack
    */
   public static boolean computeRequiredRhoAndAchievableQddotGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddotAchievableToPack,
         DenseMatrix64F rhoToPack)
   {
      if (checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack))
         return false;

      dynamicsMatrixCalculator.getFloatingBaseMassMatrix(localMassMatrix);
      dynamicsMatrixCalculator.getFloatingBaseCoriolisMatrix(localCoriolisMatrix);
      dynamicsMatrixCalculator.getFloatingBaseContactForceJacobian(localContactJacobian);

      matrixTranspose.reshape(localContactJacobian.getNumCols(), localContactJacobian.getNumRows());
      matrixTranspose.zero();
      CommonOps.transpose(localContactJacobian, matrixTranspose);

      CommonOps.multAdd(localMassMatrix, qddotAchievableToPack, localCoriolisMatrix);

      pseudoInverseSolver.setA(matrixTranspose);
      pseudoInverseSolver.solve(localCoriolisMatrix, rhoToPack);

      if (!checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack))
      {
         computeRequiredQddotGivenRho(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack);
         computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack);
      }

      return true;
   }

   /**
    * <p>
    * Computes the required contact forces given the joint accelerations using the rigid-body dynamics for the floating body.
    * </p>
    *
    * @param dynamicsMatrixCalculator
    * @param qddot
    * @param rhoToPack
    */
   public static boolean computeRequiredRhoGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F rhoToPack)
   {
      if (checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddot, rhoToPack))
         return false;

      dynamicsMatrixCalculator.getFloatingBaseMassMatrix(localMassMatrix);
      dynamicsMatrixCalculator.getFloatingBaseCoriolisMatrix(localCoriolisMatrix);
      dynamicsMatrixCalculator.getFloatingBaseContactForceJacobian(localContactJacobian);

      matrixTranspose.reshape(localContactJacobian.getNumCols(), localContactJacobian.getNumRows());
      matrixTranspose.zero();
      CommonOps.transpose(localContactJacobian, matrixTranspose);

      CommonOps.multAdd(localMassMatrix, qddot, localCoriolisMatrix);

      pseudoInverseSolver.setA(matrixTranspose);
      pseudoInverseSolver.solve(localCoriolisMatrix, rhoToPack);

      return true;
   }

   /**
    * <p>
    * Computes the required joint accelerations given the contact forces using the rigid-body dynamics for the floating body.
    * </p>
    *
    * @param dynamicsMatrixCalculator
    * @param qddotToPack
    * @param rho
    */
   public static boolean computeRequiredQddotGivenRho(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddotToPack, DenseMatrix64F rho)
   {
      if (checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddotToPack, rho))
         return false;

      dynamicsMatrixCalculator.getFloatingBaseMassMatrix(localMassMatrix);
      dynamicsMatrixCalculator.getFloatingBaseCoriolisMatrix(localCoriolisMatrix);
      dynamicsMatrixCalculator.getFloatingBaseContactForceJacobian(localContactJacobian);

      matrixTranspose.reshape(localContactJacobian.getNumCols(), localContactJacobian.getNumRows());
      matrixTranspose.zero();
      CommonOps.transpose(localContactJacobian, matrixTranspose);

      CommonOps.multAdd(matrixTranspose, rho, localCoriolisMatrix);

      pseudoInverseSolver.setA(localMassMatrix);
      pseudoInverseSolver.solve(localCoriolisMatrix, qddotToPack);

      return true;
   }

   public static boolean checkFloatingBaseRigidBodyDynamicsSatisfied(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F tau,
         DenseMatrix64F rho)
   {
      if (!checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddot, rho))
         return false;
      if (!checkRigidBodyDynamicsSatisfied(dynamicsMatrixCalculator, qddot, tau, rho))
         return false;

      return true;
   }

   /**
    * <p>
    *    Checks whether or not the floating base portion of the rigid body dynamics is satisfied by the given qddot and rho.
    * </p>
    * <p>
    *    Must satisfy the equation
    *    H_f*qddot + C_f = J_c,f^T rho
    * </p>
    * @param dynamicsMatrixCalculator
    * @param qddot
    * @param rho
    * @return
    */
   public static boolean checkFloatingBaseDynamicsSatisfied(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F rho)
   {
      dynamicsMatrixCalculator.getFloatingBaseMassMatrix(localMassMatrix);
      dynamicsMatrixCalculator.getFloatingBaseCoriolisMatrix(localCoriolisMatrix);
      dynamicsMatrixCalculator.getFloatingBaseContactForceJacobian(localContactJacobian);

      matrixTranspose.reshape(localContactJacobian.getNumCols(), localContactJacobian.getNumRows());
      matrixTranspose.zero();
      CommonOps.transpose(localContactJacobian, matrixTranspose);

      CommonOps.multAdd(localMassMatrix, qddot, localCoriolisMatrix);
      CommonOps.multAdd(-1.0, matrixTranspose, rho, localCoriolisMatrix);

      return equalsZero(localCoriolisMatrix, 0.01);
   }

   /**
    * <p>
    *    Checks whether or not the body portion of the rigid body dynamics is satisfied by the given qddot, tau and rho.
    * </p>
    * <p>
    *    Must satisfy the equation
    *    H_b*qddot + C_b = tau + J_c,b^T rho
    * </p>
    * @param dynamicsMatrixCalculator
    * @param qddot
    * @param rho
    * @return
    */
   public static boolean checkRigidBodyDynamicsSatisfied(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F tau, DenseMatrix64F rho)
   {
      dynamicsMatrixCalculator.getBodyMassMatrix(localMassMatrix);
      dynamicsMatrixCalculator.getBodyCoriolisMatrix(localCoriolisMatrix);
      dynamicsMatrixCalculator.getBodyContactForceJacobian(localContactJacobian);

      matrixTranspose.reshape(localContactJacobian.getNumCols(), localContactJacobian.getNumRows());
      matrixTranspose.zero();
      CommonOps.transpose(localContactJacobian, matrixTranspose);

      CommonOps.multAdd(localMassMatrix, qddot, localCoriolisMatrix);
      CommonOps.multAdd(-1.0, matrixTranspose, rho, localCoriolisMatrix);
      CommonOps.subtractEquals(localCoriolisMatrix, tau);

      return equalsZero(localCoriolisMatrix, tolerance);
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
