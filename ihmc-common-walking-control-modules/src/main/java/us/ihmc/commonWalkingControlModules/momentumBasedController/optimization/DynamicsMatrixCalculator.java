package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.FloatingBaseRigidBodyDynamicsCalculator;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;

public class DynamicsMatrixCalculator
{
   private final CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;
   private final GravityCoriolisExternalWrenchMatrixCalculator coriolisMatrixCalculator;
   private final ContactWrenchMatrixCalculator contactWrenchMatrixCalculator;

   private final DynamicsMatrixCalculatorHelper helper;

   private final DMatrixRMaj coriolisMatrix;
   private final DMatrixRMaj contactForceJacobian;

   private final DMatrixRMaj floatingBaseMassMatrix;
   private final DMatrixRMaj floatingBaseCoriolisMatrix;
   private final DMatrixRMaj floatingBaseContactForceJacobian;

   private final DMatrixRMaj bodyMassMatrix;
   private final DMatrixRMaj bodyCoriolisMatrix;
   private final DMatrixRMaj bodyContactForceJacobian;
   private final DMatrixRMaj bodyContactForceJacobianTranspose;

   private final DMatrixRMaj jointTorques;

   private final DMatrixRMaj torqueMinimizationObjective;

   private final int bodyDoFs;

   public DynamicsMatrixCalculator(WholeBodyControlCoreToolbox toolbox)
   {
      FloatingJointBasics rootJoint = toolbox.getRootJoint();
      int rhoSize = toolbox.getRhoSize();

      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();

      massMatrixCalculator = toolbox.getMassMatrixCalculator();
      coriolisMatrixCalculator = toolbox.getGravityCoriolisExternalWrenchMatrixCalculator();
      contactWrenchMatrixCalculator = toolbox.getContactWrenchMatrixCalculator();

      helper = new DynamicsMatrixCalculatorHelper(coriolisMatrixCalculator, jointIndexHandler);
      helper.setRhoSize(rhoSize);

      int numberOfDoFs = jointIndexHandler.getNumberOfDoFs();
      int floatingBaseDoFs = rootJoint != null ? rootJoint.getDegreesOfFreedom() : 0;
      bodyDoFs = numberOfDoFs - floatingBaseDoFs;

      jointTorques = new DMatrixRMaj(bodyDoFs, 1);

      coriolisMatrix = new DMatrixRMaj(numberOfDoFs, 1);
      contactForceJacobian = new DMatrixRMaj(rhoSize, numberOfDoFs);

      floatingBaseMassMatrix = new DMatrixRMaj(floatingBaseDoFs, numberOfDoFs);
      floatingBaseCoriolisMatrix = new DMatrixRMaj(floatingBaseDoFs, 1);
      floatingBaseContactForceJacobian = new DMatrixRMaj(rhoSize, floatingBaseDoFs);

      bodyMassMatrix = new DMatrixRMaj(bodyDoFs, numberOfDoFs);
      bodyCoriolisMatrix = new DMatrixRMaj(bodyDoFs, 1);
      bodyContactForceJacobian = new DMatrixRMaj(rhoSize, bodyDoFs);
      bodyContactForceJacobianTranspose = new DMatrixRMaj(bodyDoFs, rhoSize);

      torqueMinimizationObjective = new DMatrixRMaj(bodyDoFs, 1);
   }

   public void reset()
   {
      coriolisMatrixCalculator.setExternalWrenchesToZero();
   }

   public void compute()
   {
      massMatrixCalculator.reset();
      coriolisMatrixCalculator.compute();

      computeMatrices();
      computeTorqueMinimizationTaskMatrices();
   }

   /**
    * <p>
    * Sets an external force to be achieved. This is not a contactable body to use for balancing.
    * </p>
    *
    * @param rigidBody      body to which the wrench is applied.
    * @param externalWrench external wrench acting on body.
    */
   public void setExternalWrench(RigidBodyBasics rigidBody, WrenchReadOnly externalWrench)
   {
      coriolisMatrixCalculator.setExternalWrench(rigidBody, externalWrench);
   }

   private void computeMatrices()
   {
      DMatrixRMaj massMatrix = massMatrixCalculator.getMassMatrix();
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
      CommonOps_DDRM.transpose(bodyContactForceJacobian, bodyContactForceJacobianTranspose);

      torqueMinimizationObjective.set(bodyCoriolisMatrix);
      CommonOps_DDRM.scale(-1.0, torqueMinimizationObjective);
   }

   public void getFloatingBaseMassMatrix(DMatrixRMaj floatingBaseMassMatrixToPack)
   {
      floatingBaseMassMatrixToPack.set(floatingBaseMassMatrix);
   }

   public void getFloatingBaseCoriolisMatrix(DMatrixRMaj floatingBaseCoriolisMatrixToPack)
   {
      floatingBaseCoriolisMatrixToPack.set(floatingBaseCoriolisMatrix);
   }

   public void getFloatingBaseContactForceJacobian(DMatrixRMaj floatingBaseContactForceJacobianToPack)
   {
      floatingBaseContactForceJacobianToPack.set(floatingBaseContactForceJacobian);
   }

   public void getBodyMassMatrix(DMatrixRMaj bodyMassMatrixToPack)
   {
      bodyMassMatrixToPack.set(bodyMassMatrix);
   }

   public void getBodyCoriolisMatrix(DMatrixRMaj bodyCoriolisMatrixToPack)
   {
      bodyCoriolisMatrixToPack.set(bodyCoriolisMatrix);
   }

   public void getBodyContactForceJacobian(DMatrixRMaj bodyContactForceJacobianToPack)
   {
      bodyContactForceJacobianToPack.set(bodyContactForceJacobian);
   }

   public void getMassMatrix(DMatrixRMaj massMatrixToPack)
   {
      MatrixTools.setMatrixBlock(massMatrixToPack,
                                 0,
                                 0,
                                 floatingBaseMassMatrix,
                                 0,
                                 0,
                                 floatingBaseMassMatrix.getNumRows(),
                                 floatingBaseMassMatrix.getNumCols(),
                                 1.0);
      MatrixTools.setMatrixBlock(massMatrixToPack,
                                 floatingBaseMassMatrix.getNumRows(),
                                 0,
                                 bodyMassMatrix,
                                 0,
                                 0,
                                 bodyMassMatrix.getNumRows(),
                                 bodyMassMatrix.getNumCols(),
                                 1.0);
   }

   public void getCoriolisMatrix(DMatrixRMaj coriolisMatrixToPack)
   {
      MatrixTools.setMatrixBlock(coriolisMatrixToPack,
                                 0,
                                 0,
                                 floatingBaseCoriolisMatrix,
                                 0,
                                 0,
                                 floatingBaseCoriolisMatrix.getNumRows(),
                                 floatingBaseCoriolisMatrix.getNumCols(),
                                 1.0);
      MatrixTools.setMatrixBlock(coriolisMatrixToPack,
                                 floatingBaseCoriolisMatrix.getNumRows(),
                                 0,
                                 bodyCoriolisMatrix,
                                 0,
                                 0,
                                 bodyCoriolisMatrix.getNumRows(),
                                 bodyCoriolisMatrix.getNumCols(),
                                 1.0);
   }

   public DMatrixRMaj computeJointTorques(DMatrixRMaj jointAccelerationSolution, DMatrixRMaj contactForceSolution)
   {
      computeJointTorques(jointTorques, jointAccelerationSolution, contactForceSolution);

      return jointTorques;
   }

   public DMatrixRMaj getBodyMassMatrix()
   {
      return bodyMassMatrix;
   }

   public DMatrixRMaj getBodyContactForceJacobianTranspose()
   {
      return bodyContactForceJacobianTranspose;
   }

   public DMatrixRMaj getTorqueMinimizationObjective()
   {
      return torqueMinimizationObjective;
   }

   public DMatrixRMaj getBodyGravityCoriolisMatrix()
   {
      return bodyCoriolisMatrix;
   }

   private static final int large = 1000;

   private final DMatrixRMaj localBodyMassMatrix = new DMatrixRMaj(large, large);
   private final DMatrixRMaj localBodyCoriolisMatrix = new DMatrixRMaj(large, large);
   private final DMatrixRMaj localBodyContactJacobian = new DMatrixRMaj(large, large);

   private final DMatrixRMaj localFloatingMassMatrix = new DMatrixRMaj(large, large);
   private final DMatrixRMaj localFloatingCoriolisMatrix = new DMatrixRMaj(large, large);
   private final DMatrixRMaj localFloatingContactJacobian = new DMatrixRMaj(large, large);

   private final DMatrixRMaj tmpMatrix = new DMatrixRMaj(SpatialForce.SIZE);

   private final FloatingBaseRigidBodyDynamicsCalculator rbdCalculator = new FloatingBaseRigidBodyDynamicsCalculator();

   /**
    * <p>
    * Computes the joint torques that satisfy the rigid body dynamics for the desired joint
    * accelerations and contact forces
    * </p>
    *
    * @param jointTorquesToPack
    * @param jointAccelerationSolution
    * @param contactForceSolution
    */
   public void computeJointTorques(DMatrixRMaj jointTorquesToPack, DMatrixRMaj jointAccelerationSolution, DMatrixRMaj contactForceSolution)
   {
      rbdCalculator.computeTauGivenRhoAndQddot(bodyMassMatrix,
                                               bodyCoriolisMatrix,
                                               bodyContactForceJacobian,
                                               jointAccelerationSolution,
                                               contactForceSolution,
                                               jointTorquesToPack);
   }

   /**
    * <p>
    * Computes the required joint accelerations given the contact forces using the rigid-body dynamics
    * for the floating body.
    * </p>
    *
    * @param dynamicsMatrixCalculator
    * @param qddotToPack
    * @param rho
    */
   public boolean computeQddotGivenRho(DynamicsMatrixCalculator dynamicsMatrixCalculator, DMatrixRMaj qddotToPack, DMatrixRMaj rho)
   {
      if (checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddotToPack, rho))
         return false;

      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      rbdCalculator.computeQddotGivenRho(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, qddotToPack, rho);

      return true;
   }

   /**
    * <p>
    * Computes the required contact forces given the joint accelerations using the rigid-body dynamics
    * for the floating body.
    * </p>
    *
    * @param dynamicsMatrixCalculator
    * @param qddot
    * @param rhoToPack
    */
   public boolean computeRhoGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DMatrixRMaj qddot, DMatrixRMaj rhoToPack)
   {
      if (checkFloatingBaseDynamicsSatisfied(dynamicsMatrixCalculator, qddot, rhoToPack))
         return false;

      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      rbdCalculator.computeRhoGivenQddot(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, qddot, rhoToPack);

      return true;
   }

   public void computeTauGivenRhoAndQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DMatrixRMaj qddot, DMatrixRMaj rho, DMatrixRMaj tauToPack)
   {
      getBodyMatrices(dynamicsMatrixCalculator);
      rbdCalculator.computeTauGivenRhoAndQddot(localBodyMassMatrix, localBodyCoriolisMatrix, localBodyContactJacobian, qddot, rho, tauToPack);
   }

   public void computeTauGivenRho(DynamicsMatrixCalculator dynamicsMatrixCalculator, DMatrixRMaj rho, DMatrixRMaj tauToPack)
   {
      getBodyMatrices(dynamicsMatrixCalculator);
      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      rbdCalculator.computeTauGivenRho(localFloatingMassMatrix,
                                       localFloatingCoriolisMatrix,
                                       localFloatingContactJacobian,
                                       localBodyMassMatrix,
                                       localBodyCoriolisMatrix,
                                       localBodyContactJacobian,
                                       rho,
                                       tauToPack);
   }

   public void computeTauGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DMatrixRMaj qddot, DMatrixRMaj tauToPack)
   {
      getBodyMatrices(dynamicsMatrixCalculator);
      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      rbdCalculator.computeTauGivenQddot(localFloatingMassMatrix,
                                         localFloatingCoriolisMatrix,
                                         localFloatingContactJacobian,
                                         localBodyMassMatrix,
                                         localBodyCoriolisMatrix,
                                         localBodyContactJacobian,
                                         qddot,
                                         tauToPack);
   }

   /**
    * <p>
    * Computes the required contact forces given the joint accelerations using the rigid-body dynamics
    * for the floating body.
    * </p>
    *
    * @param dynamicsMatrixCalculator
    * @param qddotAchievableToPack
    * @param rhoToPack
    */
   public boolean computeRequiredRhoAndAchievableQddotGivenRho(DynamicsMatrixCalculator dynamicsMatrixCalculator, DMatrixRMaj qddotAchievableToPack,
                                                               DMatrixRMaj rhoToPack)
   {
      return computeRequiredRhoAndAchievableQddotGivenRho(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack, 0);
   }

   private boolean computeRequiredRhoAndAchievableQddotGivenRho(DynamicsMatrixCalculator dynamicsMatrixCalculator, DMatrixRMaj qddotAchievableToPack,
                                                                DMatrixRMaj rhoToPack, int iter)
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
    * Computes the required contact forces given the joint accelerations using the rigid-body dynamics
    * for the floating body.
    * </p>
    *
    * @param dynamicsMatrixCalculator
    * @param qddotAchievableToPack
    * @param rhoToPack
    */
   public boolean computeRequiredRhoAndAchievableQddotGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DMatrixRMaj qddotAchievableToPack,
                                                                 DMatrixRMaj rhoToPack)
   {
      return computeRequiredRhoAndAchievableQddotGivenQddot(dynamicsMatrixCalculator, qddotAchievableToPack, rhoToPack, 0);
   }

   private boolean computeRequiredRhoAndAchievableQddotGivenQddot(DynamicsMatrixCalculator dynamicsMatrixCalculator, DMatrixRMaj qddotAchievableToPack,
                                                                  DMatrixRMaj rhoToPack, int iter)
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

   public CompositeRigidBodyMassMatrixCalculator getMassMatrixCalculator()
   {
      return massMatrixCalculator;
   }

   public boolean checkFloatingBaseRigidBodyDynamicsSatisfied(DynamicsMatrixCalculator dynamicsMatrixCalculator, DMatrixRMaj qddot, DMatrixRMaj tau,
                                                              DMatrixRMaj rho)
   {
      getBodyMatrices(dynamicsMatrixCalculator);
      getFloatingBaseMatrices(dynamicsMatrixCalculator);

      return rbdCalculator.areFloatingBaseRigidBodyDynamicsSatisfied(localFloatingMassMatrix,
                                                                     localFloatingCoriolisMatrix,
                                                                     localFloatingContactJacobian,
                                                                     localBodyMassMatrix,
                                                                     localBodyCoriolisMatrix,
                                                                     localBodyContactJacobian,
                                                                     qddot,
                                                                     tau,
                                                                     rho);
   }

   public void extractTorqueMatrix(JointBasics[] joints, DMatrixRMaj torqueMatrixToPack)
   {
      OneDoFJointBasics[] filteredJoints = MultiBodySystemTools.filterJoints(joints, RevoluteJoint.class);
      int bodyDoFs = MultiBodySystemTools.computeDegreesOfFreedom(filteredJoints);

      int startIndex = 0;
      for (int i = 0; i < bodyDoFs; i++)
      {
         JointBasics joint = filteredJoints[i];
         int jointDoF = joint.getDegreesOfFreedom();
         tmpMatrix.reshape(jointDoF, 1);
         joint.getJointTau(0, tmpMatrix);

         for (int dof = 0; dof < jointDoF; dof++)
            torqueMatrixToPack.set(startIndex + dof, 0, tmpMatrix.get(dof, 0));
         startIndex += jointDoF;
      }

   }

   /**
    * <p>
    * Checks whether or not the floating base portion of the rigid body dynamics is satisfied by the
    * given qddot and rho.
    * </p>
    * <p>
    * Must satisfy the equation H_f*qddot + C_f = J_c,f^T rho
    * </p>
    * 
    * @param dynamicsMatrixCalculator
    * @param qddot
    * @param rho
    * @return
    */
   public boolean checkFloatingBaseDynamicsSatisfied(DynamicsMatrixCalculator dynamicsMatrixCalculator, DMatrixRMaj qddot, DMatrixRMaj rho)
   {
      getFloatingBaseMatrices(dynamicsMatrixCalculator);
      return rbdCalculator.areFloatingBaseDynamicsSatisfied(localFloatingMassMatrix, localFloatingCoriolisMatrix, localFloatingContactJacobian, qddot, rho);
   }

   /**
    * <p>
    * Checks whether or not the body portion of the rigid body dynamics is satisfied by the given
    * qddot, tau and rho.
    * </p>
    * <p>
    * Must satisfy the equation H_b*qddot + C_b = tau + J_c,b^T rho
    * </p>
    * 
    * @param dynamicsMatrixCalculator
    * @param qddot
    * @param rho
    * @return
    */
   public boolean checkRigidBodyDynamicsSatisfied(DynamicsMatrixCalculator dynamicsMatrixCalculator, DMatrixRMaj qddot, DMatrixRMaj tau,
                                                  DMatrixRMaj rho)
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
