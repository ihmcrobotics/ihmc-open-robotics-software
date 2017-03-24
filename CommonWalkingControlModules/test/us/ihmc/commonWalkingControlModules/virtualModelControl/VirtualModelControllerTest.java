package us.ihmc.commonWalkingControlModules.virtualModelControl;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.After;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControllerTestHelper.RobotLegs;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class VirtualModelControllerTest
{
   private final Random bigRandom = new Random(1000L);
   private final Random random = new Random();
   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private boolean hasSCSSimulation = false;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testJacobianCalculation()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench wrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      InverseDynamicsJoint[] controlledJoints = ScrewTools.createJointPath(pelvis, foot);
      GeometricJacobian jacobian = new GeometricJacobian(controlledJoints, pelvis.getBodyFixedFrame());
      jacobian.compute();

      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      DenseMatrix64F transposeJacobianMatrix = new DenseMatrix64F(jacobianMatrix.numCols, Wrench.SIZE);
      CommonOps.transpose(jacobianMatrix, transposeJacobianMatrix);

      wrench.changeFrame(pelvis.getBodyFixedFrame());
      DenseMatrix64F wrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      wrenchMatrix.set(0, 0, wrench.getAngularPartX());
      wrenchMatrix.set(1, 0, wrench.getAngularPartY());
      wrenchMatrix.set(2, 0, wrench.getAngularPartZ());
      wrenchMatrix.set(3, 0, wrench.getLinearPartX());
      wrenchMatrix.set(4, 0, wrench.getLinearPartY());
      wrenchMatrix.set(5, 0, wrench.getLinearPartZ());

      DenseMatrix64F jointEffort = new DenseMatrix64F(controlledJoints.length, 1);
      CommonOps.multTransA(jacobianMatrix, wrenchMatrix, jointEffort);

      desiredForce.changeFrame(foot.getBodyFixedFrame());
      wrench.changeFrame(foot.getBodyFixedFrame());

      DenseMatrix64F appliedWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      CommonOps.invert(transposeJacobianMatrix);
      CommonOps.mult(transposeJacobianMatrix, jointEffort, appliedWrenchMatrix);
      Wrench appliedWrench = new Wrench(foot.getBodyFixedFrame(), jacobian.getJacobianFrame(), appliedWrenchMatrix);
      appliedWrench.changeFrame(foot.getBodyFixedFrame());

      VirtualModelControllerTestHelper.compareWrenches(wrench, appliedWrench);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMC()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame and no selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      submitAndCheckVMC(pelvis, foot, desiredWrench, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testVMCSelectAll()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      submitAndCheckVMC(pelvis, foot, desiredWrench, CommonOps.identity(Wrench.SIZE, Wrench.SIZE));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCSelectForce()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only force
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 3, 1);
      selectionMatrix.set(1, 4, 1);
      selectionMatrix.set(2, 5, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCSelectTorque()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 1, 1);
      selectionMatrix.set(2, 2, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCSelectForceX()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCSelectForceY()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 4, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCSelectForceZ()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 5, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCSelectTorqueX()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 0, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCSelectTorqueY()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 1, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCSelectTorqueZ()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 2, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCSelectForceXTorqueY()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(2, 6);
      selectionMatrix.set(0, 1, 1);
      selectionMatrix.set(1, 4, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCSelectForceYZTorqueX()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 4, 1);
      selectionMatrix.set(2, 5, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCSelectForceXTorqueXZ()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 2, 1);
      selectionMatrix.set(2, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCWrongExpressedInFrame()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), pelvis.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 2, 1);
      selectionMatrix.set(2, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCWrongExpressedOnFrame()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(pelvis.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 2, 1);
      selectionMatrix.set(2, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCWrongExpressedInAndOnFrame()
   {
      double gravity = -9.81;

      
      RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(pelvis.getBodyFixedFrame(), pelvis.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 2, 1);
      selectionMatrix.set(2, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVMCVirtualWrenchCommand()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      
      VirtualModelControllerTestHelper.RobotLegs robotLeg = VirtualModelControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3D(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      DenseMatrix64F selectionMatrix = CommonOps.identity(Wrench.SIZE, Wrench.SIZE);

      InverseDynamicsJoint[] controlledJoints = ScrewTools.createJointPath(pelvis, endEffector);
      GeometricJacobian jacobian = new GeometricJacobian(controlledJoints, pelvis.getBodyFixedFrame());
      jacobian.compute();

      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      DenseMatrix64F transposeJacobianMatrix = new DenseMatrix64F(jacobianMatrix.numCols, Wrench.SIZE);
      CommonOps.transpose(jacobianMatrix, transposeJacobianMatrix);
      CommonOps.invert(transposeJacobianMatrix);

      YoVariableRegistry registry = new YoVariableRegistry(this.getClass().getSimpleName());
      VirtualModelController virtualModelController = new VirtualModelController(new GeometricJacobianHolder(), pelvis, null, registry, null);
      virtualModelController.registerControlledBody(endEffector, pelvis);

      VirtualWrenchCommand virtualWrenchCommand = new VirtualWrenchCommand();
      virtualWrenchCommand.set(foot, desiredWrench, selectionMatrix);

      virtualModelController.submitControlledBodyVirtualWrench(virtualWrenchCommand);

      // find jacobian transpose solution
      VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();
      virtualModelController.compute(virtualModelControlSolution);

      desiredWrench.changeFrame(pelvis.getBodyFixedFrame());

      // compute end effector force from torques
      Map<InverseDynamicsJoint, Double> jointTorques = virtualModelControlSolution.getJointTorques();
      DenseMatrix64F jointEffortMatrix = new DenseMatrix64F(controlledJoints.length, 1);
      for (int i = 0; i < controlledJoints.length; i++)
      {
         jointEffortMatrix.set(i, 0, jointTorques.get(controlledJoints[i]));
      }

      DenseMatrix64F appliedWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      CommonOps.mult(transposeJacobianMatrix, jointEffortMatrix, appliedWrenchMatrix);
      Wrench appliedWrench = new Wrench(endEffector.getBodyFixedFrame(), jacobian.getJacobianFrame(), appliedWrenchMatrix);

      VirtualModelControllerTestHelper.compareWrenches(desiredWrench, appliedWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 4.1)
   @Test(timeout = 30000)
   public void testVMCWithArm() throws Exception
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      
      VirtualModelControllerTestHelper.RobotArm robotArm = VirtualModelControllerTestHelper.createRobotArm();
      SCSRobotFromInverseDynamicsRobotModel scsRobotArm = robotArm.getSCSRobotArm();

      List<RigidBody> endEffectors = new ArrayList<>();
      RigidBody hand = robotArm.getHand();
      endEffectors.add(hand);

      double forceX = random.nextDouble() * 10.0;
      double forceY = random.nextDouble() * 10.0;
      double forceZ = random.nextDouble() * 10.0;
      double torqueX = random.nextDouble() * 10.0;
      double torqueY = random.nextDouble() * 10.0;
      double torqueZ = random.nextDouble() * 10.0;
      Vector3D desiredForce = new Vector3D(forceX, forceY, forceZ);
      Vector3D desiredTorque = new Vector3D(torqueX, torqueY, torqueZ);

      List<Vector3D> desiredForces = new ArrayList<>();
      List<Vector3D> desiredTorques = new ArrayList<>();
      desiredForces.add(desiredForce);
      desiredTorques.add(desiredTorque);

      List<ExternalForcePoint> externalForcePoints = new ArrayList<>();
      externalForcePoints.add(robotArm.getExternalForcePoint());

      DenseMatrix64F selectionMatrix = CommonOps.identity(Wrench.SIZE, Wrench.SIZE);
      VirtualModelControllerTestHelper.createVirtualModelControlTest(scsRobotArm, robotArm, robotArm.getCenterOfMassFrame(), endEffectors, desiredForces,
            desiredTorques, externalForcePoints, selectionMatrix, simulationTestingParameters);

      simulationTestingParameters.setKeepSCSUp(false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 2.8)
   @Test(timeout = 30000)
   public void testVMCWithPlanarArm() throws Exception
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      
      VirtualModelControllerTestHelper.PlanarRobotArm robotArm = VirtualModelControllerTestHelper.createPlanarArm();
      SCSRobotFromInverseDynamicsRobotModel scsRobotArm = robotArm.getSCSRobotArm();

      List<RigidBody> endEffectors = new ArrayList<>();
      RigidBody hand = robotArm.getHand();
      endEffectors.add(hand);

      double forceX = random.nextDouble() * 10.0;
      double forceZ = random.nextDouble() * 10.0;
      double torqueY = random.nextDouble() * 10.0;
      Vector3D desiredForce = new Vector3D(forceX, 0.0, forceZ);
      Vector3D desiredTorque = new Vector3D(0.0, torqueY, 0.0);

      List<Vector3D> desiredForces = new ArrayList<>();
      List<Vector3D> desiredTorques = new ArrayList<>();
      desiredForces.add(desiredForce);
      desiredTorques.add(desiredTorque);

      List<ExternalForcePoint> externalForcePoints = new ArrayList<>();
      externalForcePoints.add(robotArm.getExternalForcePoint());

      DenseMatrix64F selectionMatrix = CommonOps.identity(Wrench.SIZE, Wrench.SIZE);
      VirtualModelControllerTestHelper.createVirtualModelControlTest(scsRobotArm, robotArm, robotArm.getCenterOfMassFrame(), endEffectors, desiredForces, desiredTorques, externalForcePoints, selectionMatrix, simulationTestingParameters);

      simulationTestingParameters.setKeepSCSUp(false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 4.7)
   @Test(timeout = 30000)
   public void testPlanarHydra() throws Exception
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      
      VirtualModelControllerTestHelper.PlanarForkedRobotArm robotArm = VirtualModelControllerTestHelper.createPlanarForkedRobotArm();
      SCSRobotFromInverseDynamicsRobotModel scsRobotArm = robotArm.getSCSRobotArm();

      List<RigidBody> endEffectors = new ArrayList<>();
      RigidBody leftHand = robotArm.getHand(RobotSide.LEFT);
      RigidBody rightHand = robotArm.getHand(RobotSide.RIGHT);
      endEffectors.add(leftHand);
      endEffectors.add(rightHand);

      double forceX = random.nextDouble() * 10.0;
      double forceZ = random.nextDouble() * 10.0;
      double torqueY = random.nextDouble() * 10.0;
      Vector3D desiredForce1 = new Vector3D(forceX, 0.0, forceZ);
      Vector3D desiredForce2 = new Vector3D(forceX, 0.0, forceZ);
      Vector3D desiredTorque1 = new Vector3D(0.0, torqueY, 0.0);
      Vector3D desiredTorque2 = new Vector3D(0.0, torqueY, 0.0);

      List<Vector3D> desiredForces = new ArrayList<>();
      List<Vector3D> desiredTorques = new ArrayList<>();
      desiredForces.add(desiredForce1);
      desiredForces.add(desiredForce2);
      desiredTorques.add(desiredTorque1);
      desiredTorques.add(desiredTorque2);

      SideDependentList<ExternalForcePoint> sideDependentExternalForcePoints = robotArm.getExternalForcePoints();
      List<ExternalForcePoint> externalForcePoints = new ArrayList<>();
      externalForcePoints.add(sideDependentExternalForcePoints.get(RobotSide.LEFT));
      externalForcePoints.add(sideDependentExternalForcePoints.get(RobotSide.RIGHT));

      DenseMatrix64F selectionMatrix = CommonOps.identity(Wrench.SIZE, Wrench.SIZE);

      VirtualModelControllerTestHelper.createVirtualModelControlTest(scsRobotArm, robotArm, robotArm.getCenterOfMassFrame(), endEffectors, desiredForces,
            desiredTorques, externalForcePoints, selectionMatrix, simulationTestingParameters);

      simulationTestingParameters.setKeepSCSUp(false);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 5.5)
   @Test(timeout = 30000)
   public void testHydra() throws Exception
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      
      VirtualModelControllerTestHelper.ForkedRobotArm robotArm = VirtualModelControllerTestHelper.createForkedRobotArm();
      SCSRobotFromInverseDynamicsRobotModel scsRobotArm = robotArm.getSCSRobotArm();

      List<RigidBody> endEffectors = new ArrayList<>();
      RigidBody leftHand = robotArm.getHand(RobotSide.LEFT);
      RigidBody rightHand = robotArm.getHand(RobotSide.RIGHT);
      endEffectors.add(leftHand);
      endEffectors.add(rightHand);

      double forceZ = random.nextDouble() * 10.0;
      Vector3D desiredForce1 = new Vector3D(0.0, 0.0, forceZ);
      Vector3D desiredForce2 = new Vector3D(0.0, 0.0, forceZ);
      Vector3D desiredTorque1 = new Vector3D(0.0, 0.0, 0.0);
      Vector3D desiredTorque2 = new Vector3D(0.0, 0.0, 0.0);

      List<Vector3D> desiredForces = new ArrayList<>();
      List<Vector3D> desiredTorques = new ArrayList<>();
      desiredForces.add(desiredForce1);
      desiredForces.add(desiredForce2);
      desiredTorques.add(desiredTorque1);
      desiredTorques.add(desiredTorque2);

      SideDependentList<ExternalForcePoint> sideDependentExternalForcePoints = robotArm.getExternalForcePoints();
      List<ExternalForcePoint> externalForcePoints = new ArrayList<>();
      externalForcePoints.add(sideDependentExternalForcePoints.get(RobotSide.LEFT));
      externalForcePoints.add(sideDependentExternalForcePoints.get(RobotSide.RIGHT));

      DenseMatrix64F selectionMatrix = CommonOps.identity(Wrench.SIZE, Wrench.SIZE);

      VirtualModelControllerTestHelper.createVirtualModelControlTest(scsRobotArm, robotArm, robotArm.getCenterOfMassFrame(), endEffectors, desiredForces,
            desiredTorques, externalForcePoints, selectionMatrix, simulationTestingParameters);

      simulationTestingParameters.setKeepSCSUp(false);
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp() && hasSCSSimulation)
      {
         ThreadTools.sleepForever();
      }
   }

   private void submitAndCheckVMC(RigidBody base, RigidBody endEffector, Wrench desiredWrench, DenseMatrix64F selectionMatrix)
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      simulationTestingParameters.setKeepSCSUp(false);

      OneDoFJoint[] controlledJoints = ScrewTools.createOneDoFJointPath(base, endEffector);
      GeometricJacobian jacobian = new GeometricJacobian(controlledJoints, base.getBodyFixedFrame());
      jacobian.compute();

      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      DenseMatrix64F transposeJacobianMatrix = new DenseMatrix64F(jacobianMatrix.numCols, Wrench.SIZE);
      CommonOps.transpose(jacobianMatrix, transposeJacobianMatrix);
      CommonOps.invert(transposeJacobianMatrix);

      VirtualModelController virtualModelController = new VirtualModelController(new GeometricJacobianHolder(), base, null, registry, null);
      virtualModelController.registerControlledBody(endEffector, base);

      desiredWrench.changeFrame(base.getBodyFixedFrame());

      if (selectionMatrix == null)
         virtualModelController.submitControlledBodyVirtualWrench(endEffector, desiredWrench);
      else
         virtualModelController.submitControlledBodyVirtualWrench(endEffector, desiredWrench, selectionMatrix);

      // find jacobian transpose solution
      VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();
      virtualModelController.compute(virtualModelControlSolution);

      // compute end effector force from torques
      Map<InverseDynamicsJoint, Double> jointTorques = virtualModelControlSolution.getJointTorques();
      DenseMatrix64F jointEffortMatrix = new DenseMatrix64F(controlledJoints.length, 1);
      for (int i = 0; i < controlledJoints.length; i++)
      {
         jointEffortMatrix.set(i, 0, jointTorques.get(controlledJoints[i]));
      }

      DenseMatrix64F appliedWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      CommonOps.mult(transposeJacobianMatrix, jointEffortMatrix, appliedWrenchMatrix);
      Wrench appliedWrench = new Wrench(endEffector.getBodyFixedFrame(), jacobian.getJacobianFrame(), appliedWrenchMatrix);

      if (selectionMatrix == null)
         VirtualModelControllerTestHelper.compareWrenches(desiredWrench, appliedWrench);
      else
         VirtualModelControllerTestHelper.compareWrenches(desiredWrench, appliedWrench, selectionMatrix);
   }
}
