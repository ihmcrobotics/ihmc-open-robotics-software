package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.robotics.screwTheory.*;

public class DynamicsMatrixCalculator
{
   private final CompositeRigidBodyMassMatrixHandler massMatrixHandler;
   private final GravityCoriolisExternalWrenchMatrixCalculator coriolisMatrixCalculator;
   private final ContactWrenchMatrixCalculator contactWrenchMatrixCalculator;

   private final DynamicsMatrixCalculatorHelper helper;

   private final DenseMatrix64F coriolisMatrix;
   private final DenseMatrix64F contactForceJacobian;

   private final DenseMatrix64F floatingBaseMassMatrix;
   private final DenseMatrix64F floatingBaseCoriolisMatrix;
   private final DenseMatrix64F floatingBaseContactForceJacobian;

   private final DenseMatrix64F bodyMassMatrix;
   private final DenseMatrix64F bodyCoriolisMatrix;
   private final DenseMatrix64F bodyContactForceJacobian;
   private final DenseMatrix64F bodyContactForceJacobianTranspose;

   private final DenseMatrix64F jointTorques;

   private final DenseMatrix64F torqueMinimizationObjective;

   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final int bodyDoFs;

   public DynamicsMatrixCalculator(WholeBodyControlCoreToolbox toolbox, WrenchMatrixCalculator wrenchMatrixCalculator)
   {
      this(new ArrayList<>(), toolbox, wrenchMatrixCalculator);
   }

   public DynamicsMatrixCalculator(ArrayList<InverseDynamicsJoint> jointsToIgnore, WholeBodyControlCoreToolbox toolbox, WrenchMatrixCalculator wrenchMatrixCalculator)
   {
      FloatingInverseDynamicsJoint rootJoint = toolbox.getRootJoint();
      RigidBody rootBody = toolbox.getRootBody();
      int rhoSize = wrenchMatrixCalculator.getRhoSize();

      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();

      massMatrixHandler = new CompositeRigidBodyMassMatrixHandler(rootBody, jointsToIgnore);
      coriolisMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(rootBody, jointsToIgnore, toolbox.getGravityZ());
      contactWrenchMatrixCalculator = new ContactWrenchMatrixCalculator(rootBody, toolbox.getContactablePlaneBodies(), wrenchMatrixCalculator, jointIndexHandler);

      helper = new DynamicsMatrixCalculatorHelper(coriolisMatrixCalculator, jointIndexHandler);
      helper.setRhoSize(rhoSize);

      int numberOfDoFs = jointIndexHandler.getNumberOfDoFs();
      int floatingBaseDoFs = rootJoint != null ? rootJoint.getDegreesOfFreedom() : 0;
      bodyDoFs = numberOfDoFs - floatingBaseDoFs;

      jointTorques = new DenseMatrix64F(bodyDoFs, 1);

      coriolisMatrix = new DenseMatrix64F(numberOfDoFs, 1);
      contactForceJacobian = new DenseMatrix64F(rhoSize, numberOfDoFs);

      floatingBaseMassMatrix = new DenseMatrix64F(floatingBaseDoFs, numberOfDoFs);
      floatingBaseCoriolisMatrix = new DenseMatrix64F(floatingBaseDoFs, 1);
      floatingBaseContactForceJacobian = new DenseMatrix64F(rhoSize, floatingBaseDoFs);

      bodyMassMatrix = new DenseMatrix64F(bodyDoFs, numberOfDoFs);
      bodyCoriolisMatrix = new DenseMatrix64F(bodyDoFs, 1);
      bodyContactForceJacobian = new DenseMatrix64F(rhoSize, bodyDoFs);
      bodyContactForceJacobianTranspose = new DenseMatrix64F(bodyDoFs ,rhoSize);

      torqueMinimizationObjective = new DenseMatrix64F(bodyDoFs, 1);
   }

   public void reset()
   {
      coriolisMatrixCalculator.reset();
   }

   public void compute()
   {
      massMatrixHandler.compute();
      coriolisMatrixCalculator.compute();

      computeMatrices();
      computeTorqueMinimizationTaskMatrices();
   }

   /**
    * <p>
    * Sets an external force to be achieved. This is not a contactable body to use for balancing.
    * </p>
    *
    * @param rigidBody body to which the wrench is applied.
    * @param externalWrench external wrench acting on body.
    */
   public void setExternalWrench(RigidBody rigidBody, Wrench externalWrench)
   {
      coriolisMatrixCalculator.setExternalWrench(rigidBody, externalWrench);
   }

   private void computeMatrices()
   {
      DenseMatrix64F massMatrix = massMatrixHandler.getMassMatrix(jointsToOptimizeFor);
      helper.extractFloatingBaseMassMatrix(massMatrix, floatingBaseMassMatrix);
      helper.extractBodyMassMatrix(massMatrix, bodyMassMatrix);

      helper.computeCoriolisMatrix(coriolisMatrix);
      helper.extractFloatingBaseCoriolisMatrix(coriolisMatrix, floatingBaseCoriolisMatrix);
      helper.extractBodyCoriolisMatrix(coriolisMatrix, bodyCoriolisMatrix);

      contactWrenchMatrixCalculator.computeContactForceJacobian(contactForceJacobian);
      helper.extractFloatingBaseContactForceJacobianMatrix(contactForceJacobian, floatingBaseContactForceJacobian);
      helper.extractBodyContactForceJacobianMatrix(contactForceJacobian, bodyContactForceJacobian);
   }

   private void computeTorqueMinimizationTaskMatrices()
   {
      CommonOps.transpose(bodyContactForceJacobian, bodyContactForceJacobianTranspose);

      torqueMinimizationObjective.set(bodyCoriolisMatrix);
      CommonOps.scale(-1.0, torqueMinimizationObjective);
   }

   public void getFloatingBaseMassMatrix(DenseMatrix64F floatingBaseMassMatrixToPack)
   {
      floatingBaseMassMatrixToPack.set(floatingBaseMassMatrix);
   }

   public void getFloatingBaseCoriolisMatrix(DenseMatrix64F floatingBaseCoriolisMatrixToPack)
   {
      floatingBaseCoriolisMatrixToPack.set(floatingBaseCoriolisMatrix);
   }

   public void getFloatingBaseContactForceJacobian(DenseMatrix64F floatingBaseContactForceJacobianToPack)
   {
      floatingBaseContactForceJacobianToPack.set(floatingBaseContactForceJacobian);
   }

   public void getBodyMassMatrix(DenseMatrix64F bodyMassMatrixToPack)
   {
      bodyMassMatrixToPack.set(bodyMassMatrix);
   }

   public void getBodyCoriolisMatrix(DenseMatrix64F bodyCoriolisMatrixToPack)
   {
      bodyCoriolisMatrixToPack.set(bodyCoriolisMatrix);
   }

   public void getBodyContactForceJacobian(DenseMatrix64F bodyContactForceJacobianToPack)
   {
      bodyContactForceJacobianToPack.set(bodyContactForceJacobian);
   }

   public DenseMatrix64F computeJointTorques(DenseMatrix64F jointAccelerationSolution, DenseMatrix64F contactForceSolution)
   {
      computeJointTorques(jointTorques, jointAccelerationSolution, contactForceSolution);

      return jointTorques;
   }

   public DenseMatrix64F getTorqueMinimizationAccelerationJacobian()
   {
      return bodyMassMatrix;
   }

   public DenseMatrix64F getTorqueMinimizationRhoJacobian()
   {
      return bodyContactForceJacobianTranspose;
   }

   public DenseMatrix64F getTorqueMinimizationObjective()
   {
      return torqueMinimizationObjective;
   }

   private static final int large = 1000;

   private final DenseMatrix64F localBodyMassMatrix = new DenseMatrix64F(large, large);
   private final DenseMatrix64F localBodyCoriolisMatrix = new DenseMatrix64F(large, large);
   private final DenseMatrix64F localBodyContactJacobian = new DenseMatrix64F(large, large);

   private final DenseMatrix64F localFloatingMassMatrix = new DenseMatrix64F(large, large);
   private final DenseMatrix64F localFloatingCoriolisMatrix = new DenseMatrix64F(large, large);
   private final DenseMatrix64F localFloatingContactJacobian = new DenseMatrix64F(large, large);

   private final DenseMatrix64F tmpMatrix = new DenseMatrix64F(SpatialForceVector.SIZE);

   private final FloatingBaseRigidBodyDynamicsCalculator rbdCalculator = new FloatingBaseRigidBodyDynamicsCalculator();

   /**
    * <p>
    * Computes the joint torques that satisfy the rigid body dynamics for the desired joint accelerations and contact forces
    * </p>
    *
    * @param jointTorquesToPack
    * @param jointAccelerationSolution
    * @param contactForceSolution
    */
   public void computeJointTorques(DenseMatrix64F jointTorquesToPack, DenseMatrix64F jointAccelerationSolution, DenseMatrix64F contactForceSolution)
   {
      rbdCalculator.computeTauGivenRhoAndQddot(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobian, jointAccelerationSolution,
            contactForceSolution, jointTorquesToPack);
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
   public boolean computeQddotGivenRho(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddotToPack, DenseMatrix64F rho)
   {
      if (checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddotToPack, rho))
         return false;

      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      rbdCalculator.computeQddotGivenRho(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, qddotToPack, rho);

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
   public boolean computeRhoGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F rhoToPack)
   {
      if (checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddot, rhoToPack))
         return false;

      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      rbdCalculator.computeRhoGivenQddot(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, qddot, rhoToPack);

      return true;
   }

   public void computeTauGivenRhoAndQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F rho, DenseMatrix64F tauToPack)
   {
      getBodyMatrices(dynamicsMatrixCalculator);
      rbdCalculator.computeTauGivenRhoAndQddot(localBodyMassMatrix, localBodyCoriolisMatrix, localBodyContactJacobian, qddot, rho, tauToPack);
   }

   public void computeTauGivenRho(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F rho, DenseMatrix64F tauToPack)
   {
      getBodyMatrices(dynamicsMatrixCalculator);
      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      rbdCalculator.computeTauGivenRho(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, localBodyMassMatrix,
            localBodyCoriolisMatrix, localBodyContactJacobian, rho, tauToPack);
   }

   public void computeTauGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F tauToPack)
   {
      getBodyMatrices(dynamicsMatrixCalculator);
      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      rbdCalculator.computeTauGivenQddot(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, localBodyMassMatrix,
            localBodyCoriolisMatrix, localBodyContactJacobian, qddot, tauToPack);
   }


   /**
    * <p>
    * Computes the required contact forces given the joint accelerations using the rigid-body dynamics for the floating body.
    * </p>
    *
    * @param dynamicsMatrixCalculator
    * @param qddotAchievableToPack
    * @param rhoToPack
    */
   public boolean computeRequiredRhoAndAchievableQddotGivenRho(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddotAchievableToPack,
         DenseMatrix64F rhoToPack)
   {
      return computeRequiredRhoAndAchievableQddotGivenRho(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack, 0);
   }

   private boolean computeRequiredRhoAndAchievableQddotGivenRho(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddotAchievableToPack,
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
    * @param qddotAchievableToPack
    * @param rhoToPack
    */
   public boolean computeRequiredRhoAndAchievableQddotGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddotAchievableToPack,
         DenseMatrix64F rhoToPack)
   {
      return computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack, 0);
   }

   private boolean computeRequiredRhoAndAchievableQddotGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddotAchievableToPack,
         DenseMatrix64F rhoToPack, int iter)
   {
      if (checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack))
         return false;

      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      rbdCalculator.computeRhoGivenQddot(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, qddotAchievableToPack, rhoToPack);

      if (!checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack))
      {
         if (iter > 1000)
            throw new RuntimeException("Overflow in computation - cannot find a satisfactory qddot.");
         computeQddotGivenRho(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack);
         computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack, iter + 1);
      }

      return true;
   }


   public boolean checkFloatingBaseRigidBodyDynamicsSatisfied(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F tau,
         DenseMatrix64F rho)
   {
      getBodyMatrices(dynamicsMatrixCalculator);
      getFloatingBaseMatrices(dynamicsMatrixCalculator);

      return rbdCalculator.areFloatingBaseRigidBodyDynamicsSatisfied(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian,
            localBodyMassMatrix, localBodyCoriolisMatrix, localBodyContactJacobian, qddot, tau, rho);
   }

   public void extractTorqueMatrix(InverseDynamicsJoint[] joints, DenseMatrix64F torqueMatrixToPack)
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
   public boolean checkFloatingBaseDynamicsSatisfied(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F rho)
   {
      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      return rbdCalculator.areFloatingBaseDynamicsSatisfied(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, qddot, rho);
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
   public boolean checkRigidBodyDynamicsSatisfied(DynamicsMatrixCalculator dynamicsMatrixCalculator, DenseMatrix64F qddot, DenseMatrix64F tau, DenseMatrix64F rho)
   {
      getBodyMatrices(dynamicsMatrixCalculator);
      return rbdCalculator.areRigidBodyDynamicsSatisfied(localBodyMassMatrix, localBodyCoriolisMatrix, localBodyContactJacobian, qddot, tau, rho);
   }

   private void getFloatingBaseMatrices(DynamicsMatrixCalculator dynamicsMatrixCalculator)
   {
      dynamicsMatrixCalculator.getFloatingBaseMassMatrix(localFloatingMassMatrix);
      dynamicsMatrixCalculator.getFloatingBaseCoriolisMatrix(localFloatingCoriolisMatrix);
      dynamicsMatrixCalculator.getFloatingBaseContactForceJacobian(localFloatingContactJacobian);
   }

   private void getBodyMatrices(DynamicsMatrixCalculator dynamicsMatrixCalculator)
   {
      dynamicsMatrixCalculator.getBodyMassMatrix(localBodyMassMatrix);
      dynamicsMatrixCalculator.getBodyCoriolisMatrix(localBodyCoriolisMatrix);
      dynamicsMatrixCalculator.getBodyContactForceJacobian(localBodyContactJacobian);
   }
}

