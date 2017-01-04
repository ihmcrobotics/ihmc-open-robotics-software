package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.*;

import java.util.ArrayList;

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
      FullRobotModel fullRobotModel = toolbox.getFullRobotModel();
      RigidBody elevator = fullRobotModel.getElevator();
      int rhoSize = wrenchMatrixCalculator.getRhoSize();

      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();
      jointsToOptimizeFor = jointIndexHandler.getIndexedJoints();

      massMatrixHandler = new CompositeRigidBodyMassMatrixHandler(elevator, jointsToIgnore);
      coriolisMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(toolbox.getTwistCalculator(), jointsToIgnore, toolbox.getGravityZ());
      contactWrenchMatrixCalculator = new ContactWrenchMatrixCalculator(elevator, toolbox.getContactablePlaneBodies(), wrenchMatrixCalculator, jointIndexHandler,
            toolbox.getGeometricJacobianHolder());

      helper = new DynamicsMatrixCalculatorHelper(coriolisMatrixCalculator, jointIndexHandler);
      helper.setRhoSize(rhoSize);

      int numberOfDoFs = jointIndexHandler.getNumberOfDoFs();
      int floatingBaseDoFs = toolbox.getRobotRootJoint().getDegreesOfFreedom();
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
      FloatingBaseRigidBodyDynamicsTools.computeTauGivenRhoAndQddot(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobian, jointAccelerationSolution,
            contactForceSolution, jointTorquesToPack);
   }
}

