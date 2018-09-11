package us.ihmc.commonWalkingControlModules.capturePoint.optimization.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelMomentumController;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControllerTestHelper.RobotLegs;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControllerTestHelper.RobotArm;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControllerTestHelper.PlanarRobotArm;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControllerTestHelper.PlanarForkedRobotArm;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControllerTestHelper.ForkedRobotArm;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static junit.framework.TestCase.assertTrue;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class VirtualModelMomentumControllerTest
{
   private final static double gravity = -9.81;


   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private Random random;

   @Before
   public void setupSimulation()
   {
      random = new Random(1000L);

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }


   @After
   public void destroySimulation()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      random = null;
      ReferenceFrameTools.clearWorldFrameTree();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testVMC()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame and no selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      submitAndCheckVMC(pelvis, foot, desiredWrench, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testVMCSelectAll()
   {
         RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testVMCSelectForce()
   {
      double gravity = -9.81;


      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      // select only force
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.clearSelection();
      selectionMatrix.setToLinearSelectionOnly();

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testVMCSelectTorque()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      // select only torque
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.clearSelection();
      selectionMatrix.setToAngularSelectionOnly();

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testVMCSelectForceX()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.clearSelection();
      selectionMatrix.selectLinearX(true);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testVMCSelectForceY()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.clearSelection();
      selectionMatrix.selectLinearY(true);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testVMCSelectForceZ()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.clearSelection();
      selectionMatrix.selectLinearZ(true);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testVMCSelectTorqueX()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.clearSelection();
      selectionMatrix.selectAngularX(true);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testVMCSelectTorqueY()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.setSelectionFrame(robotLeg.getReferenceFrames().getCenterOfMassFrame());
      selectionMatrix.clearSelection();
      selectionMatrix.selectAngularY(true);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testVMCSelectTorqueZ()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.clearSelection();
      selectionMatrix.selectAngularZ(true);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testVMCSelectForceXTorqueY()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      // select only torque
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.selectLinearX(true);
      selectionMatrix.selectAngularY(true);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testVMCSelectForceYZTorqueX()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.selectLinearY(true);
      selectionMatrix.selectLinearZ(true);
      selectionMatrix.selectAngularX(true);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testVMCSelectForceXTorqueXZ()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.selectLinearX(true);
      selectionMatrix.selectAngularX(true);
      selectionMatrix.selectAngularZ(true);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testVMCWrongExpressedInFrame()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), pelvis.getBodyFixedFrame(), desiredForce, desiredTorque);

      // select only torque
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.clearSelection();
      selectionMatrix.setToAngularSelectionOnly();

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testVMCWrongExpressedOnFrame()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(pelvis.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      // select only torque
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.clearSelection();
      selectionMatrix.setToAngularSelectionOnly();

      boolean caughtException = false;
      try
      {
         submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
      }
      catch (ReferenceFrameMismatchException e)
      {
         caughtException = true;
      }

      assertTrue("Wrong frame", caughtException);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testVMCWrongExpressedInAndOnFrame()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(pelvis.getBodyFixedFrame(), pelvis.getBodyFixedFrame(), desiredForce, desiredTorque);

      // select only torque
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.clearSelection();
      selectionMatrix.setToLinearSelectionOnly();

      boolean caughtException = false;
      try
      {
         submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
      }
      catch (ReferenceFrameMismatchException e)
      {
         caughtException = true;
      }

      assertTrue("Wrong frame", caughtException);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testVMCVirtualWrenchCommand()
   {
      RobotLegs robotLeg = VirtualModelMomentumControllerTestHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector3D desiredForce = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      FrameVector3D desiredTorque = EuclidFrameRandomTools.nextFrameVector3D(random, foot.getBodyFixedFrame());
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce, desiredTorque);

      InverseDynamicsJoint[] controlledJoints = ScrewTools.createJointPath(pelvis, endEffector);
      GeometricJacobian jacobian = new GeometricJacobian(controlledJoints, pelvis.getBodyFixedFrame());
      jacobian.compute();

      JointIndexHandler jointIndexHandler = new JointIndexHandler(controlledJoints);

      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      DenseMatrix64F transposeJacobianMatrix = new DenseMatrix64F(jacobianMatrix.numCols, Wrench.SIZE);
      CommonOps.transpose(jacobianMatrix, transposeJacobianMatrix);
      CommonOps.invert(transposeJacobianMatrix);

      VirtualModelMomentumController virtualModelController = new VirtualModelMomentumController(jointIndexHandler);

      VirtualWrenchCommand virtualWrenchCommand = new VirtualWrenchCommand();
      virtualWrenchCommand.set(pelvis, foot);
      virtualWrenchCommand.setWrench(foot.getBodyFixedFrame(), desiredTorque, desiredForce);

      virtualModelController.addVirtualEffortCommand(virtualWrenchCommand);

      // find jacobian transpose solution
      VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();
      virtualModelController.populateTorqueSolution(virtualModelControlSolution);

      desiredWrench.changeFrame(pelvis.getBodyFixedFrame());

      // compute end effector force from torques
      DenseMatrix64F jointEffortMatrix = virtualModelControlSolution.getJointTorques();

      DenseMatrix64F appliedWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      CommonOps.mult(transposeJacobianMatrix, jointEffortMatrix, appliedWrenchMatrix);
      Wrench appliedWrench = new Wrench(endEffector.getBodyFixedFrame(), jacobian.getJacobianFrame(), appliedWrenchMatrix);

      VirtualModelMomentumControllerTestHelper.compareWrenches(desiredWrench, appliedWrench);
   }

   @ContinuousIntegrationTest(estimatedDuration = 6.6)
   @Test(timeout = 33000)
   public void testVMCWithArm() throws Exception
   {
      RobotArm robotArm = VirtualModelMomentumControllerTestHelper.createRobotArm();
      SCSRobotFromInverseDynamicsRobotModel scsRobotArm = robotArm.getSCSRobotArm();

      List<RigidBody> endEffectors = new ArrayList<>();
      RigidBody hand = robotArm.getHand();
      endEffectors.add(hand);

      Vector3D desiredForce = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
      Vector3D desiredTorque = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);

      List<Vector3D> desiredForces = new ArrayList<>();
      List<Vector3D> desiredTorques = new ArrayList<>();
      desiredForces.add(desiredForce);
      desiredTorques.add(desiredTorque);

      List<ExternalForcePoint> externalForcePoints = new ArrayList<>();
      externalForcePoints.add(robotArm.getExternalForcePoint());

      VirtualModelMomentumControllerTestHelper.createVirtualModelMomentumControlTest(scsRobotArm, robotArm, robotArm.getCenterOfMassFrame(), endEffectors, desiredForces,
            desiredTorques, externalForcePoints, new SelectionMatrix6D(), simulationTestingParameters);
   }

   @ContinuousIntegrationTest(estimatedDuration = 4.7)
   @Test(timeout = 30000)
   public void testVMCWithPlanarArm() throws Exception
   {
      PlanarRobotArm robotArm = VirtualModelMomentumControllerTestHelper.createPlanarArm();
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

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      VirtualModelMomentumControllerTestHelper.createVirtualModelMomentumControlTest(scsRobotArm, robotArm, robotArm.getCenterOfMassFrame(), endEffectors,
                                                                                     desiredForces, desiredTorques, externalForcePoints, selectionMatrix, simulationTestingParameters);
   }

   @ContinuousIntegrationTest(estimatedDuration = 7.0)
   @Test(timeout = 35000)
   public void testPlanarHydra() throws Exception
   {
      PlanarForkedRobotArm robotArm = VirtualModelMomentumControllerTestHelper.createPlanarForkedRobotArm();
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

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

      VirtualModelMomentumControllerTestHelper.createVirtualModelMomentumControlTest(scsRobotArm, robotArm, robotArm.getCenterOfMassFrame(), endEffectors, desiredForces,
            desiredTorques, externalForcePoints, selectionMatrix, simulationTestingParameters);
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testHydra() throws Exception
   {
      ForkedRobotArm robotArm = VirtualModelMomentumControllerTestHelper.createForkedRobotArm();
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

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

      VirtualModelMomentumControllerTestHelper.createVirtualModelMomentumControlTest(scsRobotArm, robotArm, robotArm.getCenterOfMassFrame(), endEffectors, desiredForces,
            desiredTorques, externalForcePoints, selectionMatrix, simulationTestingParameters);
   }


   private void submitAndCheckVMC(RigidBody base, RigidBody endEffector, Wrench desiredWrench, SelectionMatrix6D selectionMatrix)
   {
      OneDoFJoint[] controlledJoints = ScrewTools.createOneDoFJointPath(base, endEffector);
      GeometricJacobian jacobian = new GeometricJacobian(controlledJoints, base.getBodyFixedFrame());
      jacobian.compute();

      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      DenseMatrix64F transposeJacobianMatrix = new DenseMatrix64F(jacobianMatrix.numCols, Wrench.SIZE);
      CommonOps.transpose(jacobianMatrix, transposeJacobianMatrix);
      CommonOps.invert(transposeJacobianMatrix);

      VirtualModelMomentumController virtualModelController = new VirtualModelMomentumController(new JointIndexHandler(controlledJoints));

      desiredWrench.changeFrame(base.getBodyFixedFrame());

      VirtualWrenchCommand virtualWrenchCommand = new VirtualWrenchCommand();
      virtualWrenchCommand.set(base, endEffector);
      virtualWrenchCommand.setWrench(desiredWrench.getExpressedInFrame(), desiredWrench);

      if (selectionMatrix != null)
         virtualWrenchCommand.setSelectionMatrix(selectionMatrix);

      virtualModelController.addVirtualEffortCommand(virtualWrenchCommand);

      // find jacobian transpose solution
      VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();
      virtualModelController.populateTorqueSolution(virtualModelControlSolution);

      // compute end effector force from torques
      DenseMatrix64F jointEffortMatrix = virtualModelControlSolution.getJointTorques();

      DenseMatrix64F appliedWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      CommonOps.mult(transposeJacobianMatrix, jointEffortMatrix, appliedWrenchMatrix);
      Wrench appliedWrench = new Wrench(endEffector.getBodyFixedFrame(), jacobian.getJacobianFrame(), appliedWrenchMatrix);

      if (selectionMatrix == null)
         VirtualModelMomentumControllerTestHelper.compareWrenches(desiredWrench, appliedWrench);
      else
         VirtualModelMomentumControllerTestHelper.compareWrenches(desiredWrench, appliedWrench, selectionMatrix);
   }
}
