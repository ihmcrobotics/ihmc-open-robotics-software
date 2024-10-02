package us.ihmc.robotics.screwTheory;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.commons.MathTools;

public class FloatingBaseRigidBodyDynamicsCalculator
{
   private static final int large = 1000;
   private static final double tolerance = 0.0001;

   private final LinearSolverDense<DMatrixRMaj> pseudoInverseSolver = LinearSolverFactory_DDRM.pseudoInverse(true);

   private final DMatrixRMaj matrixTranspose = new DMatrixRMaj(large, large);

   public FloatingBaseRigidBodyDynamicsCalculator()
   {
   }

   public void computeQddotGivenRho(DMatrixRMaj floatingBaseMassMatrix,
                                    DMatrixRMaj floatingBaseCoriolisMatrix,
                                    DMatrixRMaj floatingBaseContactForceJacobianMatrix,
                                    DMatrixRMaj qddotToPack,
                                    DMatrixRMaj rho)
   {
      CommonOps_DDRM.scale(-1.0, floatingBaseCoriolisMatrix);
      CommonOps_DDRM.multAddTransA(floatingBaseContactForceJacobianMatrix, rho, floatingBaseCoriolisMatrix);

      pseudoInverseSolver.setA(floatingBaseMassMatrix);
      pseudoInverseSolver.solve(floatingBaseCoriolisMatrix, qddotToPack);
   }

   public void computeRhoGivenQddot(DMatrixRMaj floatingBaseMassMatrix,
                                    DMatrixRMaj floatingBaseCoriolisMatrix,
                                    DMatrixRMaj floatingBaseContactForceJacobianMatrix,
                                    DMatrixRMaj qddot,
                                    DMatrixRMaj rhoToPack)
   {
      computeJacobianTranspose(floatingBaseContactForceJacobianMatrix, matrixTranspose);

      CommonOps_DDRM.multAdd(floatingBaseMassMatrix, qddot, floatingBaseCoriolisMatrix);

      pseudoInverseSolver.setA(matrixTranspose);
      pseudoInverseSolver.solve(floatingBaseCoriolisMatrix, rhoToPack);
   }

   /**
    * Based on the equation
    * <p>
    *    [0; &tau;] = H(q) qDdot + C(q, qDot) + &Sigma; J<sub>c</sub><sup>T</sup> &rho;
    * </p>
    * <p>
    *    we can compute the torque given the input acceleration qDdot and contact forces rho. By ignoring the top six rows, the zero elements of the tau
    *    vector can be ignored
    * </p>
    * @param bodyMassMatrix H
    * @param bodyCoriolisMatrix C
    * @param bodyContactForceJacobianMatrix J<sub>c</sub>
    * @param jointAccelerations qDdot in the above equation
    * @param generalizedGroundReactionForces &rho; in the above equation
    * @param tauToPack &tau; in the above equation
    */
   public static void computeTauGivenRhoAndQddot(DMatrixRMaj bodyMassMatrix,
                                                 DMatrixRMaj bodyCoriolisMatrix,
                                                 DMatrixRMaj bodyContactForceJacobianMatrix,
                                                 DMatrixRMaj jointAccelerations,
                                                 DMatrixRMaj generalizedGroundReactionForces,
                                                 DMatrixRMaj tauToPack)
   {
      tauToPack.set(bodyCoriolisMatrix);
      CommonOps_DDRM.multAdd(bodyMassMatrix, jointAccelerations, tauToPack);
      CommonOps_DDRM.multAddTransA(bodyContactForceJacobianMatrix, generalizedGroundReactionForces, tauToPack);
   }

   public static boolean areFloatingBaseRigidBodyDynamicsSatisfied(DMatrixRMaj floatingBaseMassMatrix,
                                                                   DMatrixRMaj floatingBaseCoriolisMatrix,
                                                                   DMatrixRMaj floatingBaseContactForceJacobianMatrix,
                                                                   DMatrixRMaj bodyMassMatrix,
                                                                   DMatrixRMaj bodyCoriolisMatrix,
                                                                   DMatrixRMaj bodyContactForceJacobianMatrix,
                                                                   DMatrixRMaj qddot,
                                                                   DMatrixRMaj tau,
                                                                   DMatrixRMaj rho)
   {
      if (!areFloatingBaseDynamicsSatisfied(floatingBaseMassMatrix, floatingBaseCoriolisMatrix, floatingBaseContactForceJacobianMatrix, qddot, rho))
         return false;

      return areRigidBodyDynamicsSatisfied(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobianMatrix, qddot, tau, rho);
   }

   public static boolean areFloatingBaseDynamicsSatisfied(DMatrixRMaj floatingBaseMassMatrix,
                                                          DMatrixRMaj floatingBaseCoriolisMatrix,
                                                          DMatrixRMaj floatingBaseContactForceJacobianMatrix,
                                                          DMatrixRMaj qddot,
                                                          DMatrixRMaj rho)
   {
      CommonOps_DDRM.multAdd(floatingBaseMassMatrix, qddot, floatingBaseCoriolisMatrix);
      CommonOps_DDRM.multAddTransA(-1.0, floatingBaseContactForceJacobianMatrix, rho, floatingBaseCoriolisMatrix);

      return equalsZero(floatingBaseCoriolisMatrix, tolerance);
   }

   public static boolean areRigidBodyDynamicsSatisfied(DMatrixRMaj bodyMassMatrix,
                                                       DMatrixRMaj bodyCoriolisMatrix,
                                                       DMatrixRMaj bodyContactForceJacobianMatrix,
                                                       DMatrixRMaj qddot,
                                                       DMatrixRMaj tau,
                                                       DMatrixRMaj rho)
   {
      CommonOps_DDRM.multAdd(bodyMassMatrix, qddot, bodyCoriolisMatrix);
      CommonOps_DDRM.multAddTransA(-1.0, bodyContactForceJacobianMatrix, rho, bodyCoriolisMatrix);
      CommonOps_DDRM.subtractEquals(bodyCoriolisMatrix, tau);

      return equalsZero(bodyCoriolisMatrix, tolerance);
   }

   private static void computeJacobianTranspose(DMatrixRMaj jacobian, DMatrixRMaj jacobianTransposeToPack)
   {
      jacobianTransposeToPack.reshape(jacobian.getNumCols(), jacobian.getNumRows());
      jacobianTransposeToPack.zero();
      CommonOps_DDRM.transpose(jacobian, jacobianTransposeToPack);
   }

   private static boolean equalsZero(DMatrixRMaj matrix, double tolerance)
   {
      for (int element = 0; element < matrix.getNumElements(); element++)
      {
         if (!MathTools.epsilonEquals(matrix.data[element], 0.0, tolerance))
            return false;
      }

      return true;
   }
}
