package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.*;

import java.util.ArrayList;
import java.util.List;

public class DynamicsMatrixCalculator
{
   private final CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;
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

   private final DenseMatrix64F jointTorques;


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

      massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(elevator, jointsToIgnore);
      coriolisMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(toolbox.getTwistCalculator(), toolbox.getGravityZ(), jointsToIgnore);
      contactWrenchMatrixCalculator = new ContactWrenchMatrixCalculator(elevator, toolbox.getContactablePlaneBodies(), wrenchMatrixCalculator, jointIndexHandler,
            toolbox.getGeometricJacobianHolder());

      helper = new DynamicsMatrixCalculatorHelper(coriolisMatrixCalculator, jointIndexHandler);
      helper.setRhoSize(rhoSize);

      int numberOfDoFs = jointIndexHandler.getNumberOfDoFs();
      int floatingBaseDoFs = toolbox.getRobotRootJoint().getDegreesOfFreedom();
      int bodyDoFs = numberOfDoFs - floatingBaseDoFs;

      jointTorques = new DenseMatrix64F(bodyDoFs, 1);

      coriolisMatrix = new DenseMatrix64F(numberOfDoFs, 1);
      contactForceJacobian = new DenseMatrix64F(rhoSize, numberOfDoFs);

      floatingBaseMassMatrix = new DenseMatrix64F(floatingBaseDoFs, numberOfDoFs);
      floatingBaseCoriolisMatrix = new DenseMatrix64F(floatingBaseDoFs, 1);
      floatingBaseContactForceJacobian = new DenseMatrix64F(rhoSize, floatingBaseDoFs);

      bodyMassMatrix = new DenseMatrix64F(bodyDoFs, numberOfDoFs);
      bodyCoriolisMatrix = new DenseMatrix64F(bodyDoFs, 1);
      bodyContactForceJacobian = new DenseMatrix64F(rhoSize, bodyDoFs);
   }

   public void reset()
   {
      coriolisMatrixCalculator.reset();
   }

   public void compute()
   {
      massMatrixCalculator.compute();
      coriolisMatrixCalculator.compute();

      computeMatrices();
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
      DenseMatrix64F massMatrix = massMatrixCalculator.getMassMatrix();
      helper.extractFloatingBaseMassMatrix(massMatrix, floatingBaseMassMatrix);
      helper.extractBodyMassMatrix(massMatrix, bodyMassMatrix);

      helper.computeCoriolisMatrix(coriolisMatrix);
      helper.extractFloatingBaseCoriolisMatrix(coriolisMatrix, floatingBaseCoriolisMatrix);
      helper.extractBodyCoriolisMatrix(coriolisMatrix, bodyCoriolisMatrix);

      contactWrenchMatrixCalculator.computeContactForceJacobian(contactForceJacobian);
      helper.extractFloatingBaseContactForceJacobianMatrix(contactForceJacobian, floatingBaseContactForceJacobian);
      helper.extractBodyContactForceJacobianMatrix(contactForceJacobian, bodyContactForceJacobian);
   }

   public void getFloatingBaseMassMatrix(DenseMatrix64F floatingBaseMassMatrixToPack)
   {
      floatingBaseMassMatrixToPack.set(floatingBaseMassMatrix);
   }

   public DenseMatrix64F getFloatingBaseMassMatrix()
   {
      return floatingBaseMassMatrix;
   }

   public void getFloatingBaseCoriolisMatrix(DenseMatrix64F floatingBaseCoriolisMatrixToPack)
   {
      floatingBaseCoriolisMatrixToPack.set(floatingBaseCoriolisMatrix);
   }

   public DenseMatrix64F getFloatingBaseCoriolisMatrix()
   {
      return floatingBaseCoriolisMatrix;
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

