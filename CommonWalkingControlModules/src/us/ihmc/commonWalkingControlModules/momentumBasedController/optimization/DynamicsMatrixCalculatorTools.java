package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.screwTheory.*;

public class DynamicsMatrixCalculatorTools
{
   private static final int large = 1000;

   private static final DenseMatrix64F localBodyMassMatrix = new DenseMatrix64F(large, large);
   private static final DenseMatrix64F localBodyCoriolisMatrix = new DenseMatrix64F(large, large);
   private static final DenseMatrix64F localBodyContactJacobian = new DenseMatrix64F(large, large);

   private static final DenseMatrix64F localFloatingMassMatrix = new DenseMatrix64F(large, large);
   private static final DenseMatrix64F localFloatingCoriolisMatrix = new DenseMatrix64F(large, large);
   private static final DenseMatrix64F localFloatingContactJacobian = new DenseMatrix64F(large, large);

   private static final DenseMatrix64F tmpMatrix = new DenseMatrix64F(SpatialForceVector.SIZE);

   /**
    * <p>
    * Computes the required joint accelerations given the contact forces using the rigid-body dynamics for the floating body.
    * </p>
    *
    * @param dynamicsMatrixCalculator
    * @param qddotToPack
    * @param rho
    */
   public static boolean computeQddotGivenRho(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddotToPack, DenseMatrix64F rho)
   {
      if (checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddotToPack, rho))
         return false;

      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      FloatingBaseRigidBodyDynamicsTools.computeQddotGivenRho(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, qddotToPack, rho);

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
   public static boolean computeRhoGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F rhoToPack)
   {
      if (checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddot, rhoToPack))
         return false;

      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      FloatingBaseRigidBodyDynamicsTools.computeRhoGivenQddot(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, qddot, rhoToPack);

      return true;
   }

   public static void computeTauGivenRhoAndQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F rho, DenseMatrix64F tauToPack)
   {
      getBodyMatrices(dynamicsMatrixCalculator);
      FloatingBaseRigidBodyDynamicsTools.computeTauGivenRhoAndQddot(localBodyMassMatrix, localBodyCoriolisMatrix, localBodyContactJacobian, qddot, rho, tauToPack);
   }

   public static void computeTauGivenRho(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F rho, DenseMatrix64F tauToPack)
   {
      getBodyMatrices(dynamicsMatrixCalculator);
      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      FloatingBaseRigidBodyDynamicsTools.computeTauGivenRho(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, localBodyMassMatrix,
            localBodyCoriolisMatrix, localBodyContactJacobian, rho, tauToPack);
   }

   public static void computeTauGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F tauToPack)
   {
      getBodyMatrices(dynamicsMatrixCalculator);
      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      FloatingBaseRigidBodyDynamicsTools.computeTauGivenQddot(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, localBodyMassMatrix,
            localBodyCoriolisMatrix, localBodyContactJacobian, qddot, tauToPack);
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
   public static boolean computeRequiredRhoAndAchievableQddotGivenRho(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddotAchievableToPack,
                                                                        DenseMatrix64F rhoToPack)
   {
      return computeRequiredRhoAndAchievableQddotGivenRho(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack, 0);
   }

   private static boolean computeRequiredRhoAndAchievableQddotGivenRho(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddotAchievableToPack,
                                                                      DenseMatrix64F rhoToPack, int iter)
   {
      if (checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack))
         return false;

      computeQddotGivenRho(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack);

      if (!checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack))
      {
         if (iter > 1000)
            throw new RuntimeException("Overflow in computation - cannot find a satisfactory qddot.");
         computeRhoGivenQddot(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack);
         computeRequiredRhoAndAchievableQddotGivenRho(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack, iter + 1);
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
   public static boolean computeRequiredRhoAndAchievableQddotGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddotAchievableToPack,
                                                                        DenseMatrix64F rhoToPack)
   {
      return computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack, 0);
   }

   private static boolean computeRequiredRhoAndAchievableQddotGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddotAchievableToPack,
         DenseMatrix64F rhoToPack, int iter)
   {
      if (checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack))
         return false;

      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      FloatingBaseRigidBodyDynamicsTools.computeRhoGivenQddot(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, qddotAchievableToPack, rhoToPack);

      if (!checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack))
      {
         if (iter > 1000)
            throw new RuntimeException("Overflow in computation - cannot find a satisfactory qddot.");
         computeQddotGivenRho(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack);
         computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack, iter + 1);
      }

      return true;
   }


   public static boolean checkFloatingBaseRigidBodyDynamicsSatisfied(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F tau,
         DenseMatrix64F rho)
   {
      getBodyMatrices(dynamicsMatrixCalculator);
      getFloatingBaseMatrices(dynamicsMatrixCalculator);

      return FloatingBaseRigidBodyDynamicsTools.areFloatingBaseRigidBodyDynamicsSatisfied(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian,
            localBodyMassMatrix, localBodyCoriolisMatrix, localBodyContactJacobian, qddot, tau, rho);
   }

   public static void extractTorqueMatrix(InverseDynamicsJoint[] joints, DenseMatrix64F torqueMatrixToPack)
   {
      OneDoFJoint[] filteredJoints = ScrewTools.extractRevoluteJoints(joints);
      int bodyDoFs = ScrewTools.computeDegreesOfFreedom(filteredJoints);

      int startIndex = 0;
      for (int i = 0; i < bodyDoFs; i++)
      {
         InverseDynamicsJoint joint = filteredJoints[i];
         int jointDoF = joint.getDegreesOfFreedom();
         tmpMatrix.reshape(jointDoF, 1);
         joint.getTauMatrix(tmpMatrix);

         for (int dof = 0; dof < jointDoF; dof++)
            torqueMatrixToPack.set(startIndex + dof, 0, tmpMatrix.get(dof, 0));
         startIndex += jointDoF;
      }

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
      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      return FloatingBaseRigidBodyDynamicsTools.areFloatingBaseDynamicsSatisfied(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, qddot, rho);
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
      getBodyMatrices(dynamicsMatrixCalculator);
      return FloatingBaseRigidBodyDynamicsTools.areRigidBodyDynamicsSatisfied(localBodyMassMatrix, localBodyCoriolisMatrix, localBodyContactJacobian, qddot, tau, rho);
   }

   private static void getFloatingBaseMatrices(DynamicsMatrixCalculator dynamicsMatrixCalculator)
   {
      dynamicsMatrixCalculator.getFloatingBaseMassMatrix(localFloatingMassMatrix);
      dynamicsMatrixCalculator.getFloatingBaseCoriolisMatrix(localFloatingCoriolisMatrix);
      dynamicsMatrixCalculator.getFloatingBaseContactForceJacobian(localFloatingContactJacobian);
   }

   private static void getBodyMatrices(DynamicsMatrixCalculator dynamicsMatrixCalculator)
   {
      dynamicsMatrixCalculator.getBodyMassMatrix(localBodyMassMatrix);
      dynamicsMatrixCalculator.getBodyCoriolisMatrix(localBodyCoriolisMatrix);
      dynamicsMatrixCalculator.getBodyContactForceJacobian(localBodyContactJacobian);
   }
}
