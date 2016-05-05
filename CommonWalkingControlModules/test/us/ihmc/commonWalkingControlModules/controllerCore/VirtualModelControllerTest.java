package us.ihmc.commonWalkingControlModules.controllerCore;

import static org.junit.Assert.assertTrue;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.jfree.chart.block.Block;
import org.junit.After;
import org.junit.Test;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.VirtualModelControllerTestHelper.RobotLegs;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.thread.ThreadTools;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.Map;
import java.util.Random;

//@DeployableTestClass(targets = TestPlanTarget.Fast)
public class VirtualModelControllerTest
{
   private final Random bigRandom = new Random(1000L);
   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private boolean hasSCSSimulation = false;

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testJacobianCalculation()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
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

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMC()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame and no selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      submitAndCheckVMC(pelvis, foot, desiredWrench, null);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCSelectAll()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      submitAndCheckVMC(pelvis, foot, desiredWrench, CommonOps.identity(Wrench.SIZE, Wrench.SIZE));
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCSelectForce()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only force
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 3, 1);
      selectionMatrix.set(1, 4, 1);
      selectionMatrix.set(2, 5, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCSelectTorque()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 1, 1);
      selectionMatrix.set(2, 2, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCSelectForceX()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCSelectForceY()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 4, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCSelectForceZ()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 5, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCSelectTorqueX()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 0, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCSelectTorqueY()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 1, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCSelectTorqueZ()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 2, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCSelectForceXTorqueY()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(2, 6);
      selectionMatrix.set(0, 1, 1);
      selectionMatrix.set(1, 4, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCSelectForceYZTorqueX()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 4, 1);
      selectionMatrix.set(2, 5, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCSelectForceXTorqueXZ()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 2, 1);
      selectionMatrix.set(2, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCWrongExpressedInFrame()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), pelvis.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 2, 1);
      selectionMatrix.set(2, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCWrongExpressedOnFrame()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(pelvis.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 2, 1);
      selectionMatrix.set(2, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCWrongExpressedInAndOnFrame()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(pelvis.getBodyFixedFrame(), pelvis.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 2, 1);
      selectionMatrix.set(2, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 1500)
   public void testVMCVirtualWrenchCommand()
   {
      simulationTestingParameters.setKeepSCSUp(false);
      hasSCSSimulation = false;

      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLegs robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody endEffector = robotLeg.getFoot(RobotSide.LEFT);
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      DenseMatrix64F selectionMatrix = CommonOps.identity(Wrench.SIZE, Wrench.SIZE);

      InverseDynamicsJoint[] controlledJoints = ScrewTools.createJointPath(pelvis, endEffector);
      GeometricJacobian jacobian = new GeometricJacobian(controlledJoints, pelvis.getBodyFixedFrame());
      jacobian.compute();

      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      DenseMatrix64F transposeJacobianMatrix = new DenseMatrix64F(jacobianMatrix.numCols, Wrench.SIZE);
      CommonOps.transpose(jacobianMatrix, transposeJacobianMatrix);
      CommonOps.invert(transposeJacobianMatrix);

      VirtualModelController virtualModelController = new VirtualModelController(new GeometricJacobianHolder(), pelvis, null, null);
      virtualModelController.registerEndEffector(pelvis, endEffector);

      VirtualWrenchCommand virtualWrenchCommand = new VirtualWrenchCommand();
      virtualWrenchCommand.set(foot, desiredWrench, selectionMatrix);

      virtualModelController.submitEndEffectorVirtualWrench(virtualWrenchCommand);

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

   @DeployableTestMethod
   @Test(timeout = 300000000)
   public void testVMCWithArm() throws Exception
   {
      double simulationDuration = 20.0;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      //simulationTestingParameters.setKeepSCSUp(true);
      //hasSCSSimulation = true;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      VirtualModelControllerTestHelper.RobotArm robotArm = testHelper.createRobotArm();
      SCSRobotFromInverseDynamicsRobotModel scsRobotArm = robotArm.getSCSRobotArm();

      RigidBody hand = robotArm.getHand();
      ReferenceFrame handFrame = robotArm.getHandFrame();

      FramePose desiredHandPose = new FramePose(handFrame);
      desiredHandPose.setToZero();
      desiredHandPose.changeFrame(worldFrame);

      GeometricJacobianHolder geometricJacobianHolder = new GeometricJacobianHolder();
      VirtualModelController virtualModelController = new VirtualModelController(geometricJacobianHolder, robotArm.getElevator(), registry, yoGraphicsListRegistry);
      virtualModelController.registerEndEffector(robotArm.getElevator(), hand);

      Wrench desiredWrench = new Wrench(handFrame, worldFrame);
      Vector3d desiredForce = new Vector3d(6.0, 5.0, 7.0);
      //Vector3d desiredForce = new Vector3d(0.0, 0.0, 0.0);
      desiredWrench.set(new FrameVector(worldFrame, desiredForce), new FrameVector(worldFrame, new Vector3d()));

      YoWrench yoDesiredWrench = new YoWrench("desiredWrench", handFrame, ReferenceFrame.getWorldFrame(), registry);
      yoDesiredWrench.set(desiredWrench);

      Vector3d contactForce = new Vector3d();
      contactForce.set(desiredForce);
      contactForce.scale(-1.0);

      ForcePointController forcePointController = new ForcePointController(robotArm.getExternalForcePoint(), handFrame, desiredHandPose);
      forcePointController.setInitialForce(contactForce);

      DummyArmController armController = new DummyArmController(scsRobotArm, robotArm, robotArm.getOneDoFJoints(), forcePointController, virtualModelController, hand,
            yoDesiredWrench);

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters();
      SimulationConstructionSet scs = new SimulationConstructionSet(scsRobotArm, scsParameters);
      scsRobotArm.setController(armController);
      scs.getRootRegistry().addChild(registry);
      yoGraphicsListRegistry.registerYoGraphicsList(forcePointController.getYoGraphicsList());
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      /*
      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 1500.0);
      scs.startOnAThread();
      testHelper.setSCSAndCreateSimulationRunner(scs);
      ThreadTools.sleep(TimeTools.secondsToMilliSeconds(1));
      */
      scs.startOnAThread();
      scs.simulate();

      /*
      double timeIncrement = 1.0;
      while (scs.getTime() < simulationDuration)
      {
         blockingSimulationRunner.simulateAndBlock(timeIncrement);
      }
      */

      //simulationTestingParameters.setKeepSCSUp(false);
   }

   public static void testVMCWithPlanarArm() throws Exception
   {
      double simulationDuration = 20.0;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      //simulationTestingParameters.setKeepSCSUp(true);
      //hasSCSSimulation = true;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      VirtualModelControllerTestHelper.PlanarRobotArm robotArm = testHelper.createPlanarArm();
      SCSRobotFromInverseDynamicsRobotModel scsRobotArm = robotArm.getSCSRobotArm();

      RigidBody hand = robotArm.getHand();
      ReferenceFrame handFrame = robotArm.getHandFrame();

      FramePose desiredHandPose = new FramePose(handFrame);
      desiredHandPose.setToZero();
      desiredHandPose.changeFrame(worldFrame);

      GeometricJacobianHolder geometricJacobianHolder = new GeometricJacobianHolder();
      VirtualModelController virtualModelController = new VirtualModelController(geometricJacobianHolder, robotArm.getElevator(), registry, yoGraphicsListRegistry);
      virtualModelController.registerEndEffector(robotArm.getElevator(), hand);

      Wrench desiredWrench = new Wrench(handFrame, worldFrame);
      Vector3d desiredForce = new Vector3d(6.0, 0.0, 7.0);
      //Vector3d desiredForce = new Vector3d(0.0, 0.0, 0.0);
      desiredWrench.set(new FrameVector(worldFrame, desiredForce), new FrameVector(worldFrame, new Vector3d()));

      YoWrench yoDesiredWrench = new YoWrench("desiredWrench", handFrame, ReferenceFrame.getWorldFrame(), registry);
      yoDesiredWrench.set(desiredWrench);

      Vector3d contactForce = new Vector3d();
      contactForce.set(desiredForce);
      contactForce.scale(-1.0);

      ForcePointController forcePointController = new ForcePointController(robotArm.getExternalForcePoint(), handFrame, desiredHandPose);
      forcePointController.setInitialForce(contactForce);

      DummyArmController armController = new DummyArmController(scsRobotArm, robotArm, robotArm.getOneDoFJoints(), forcePointController, virtualModelController, hand,
            yoDesiredWrench);

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters();
      SimulationConstructionSet scs = new SimulationConstructionSet(scsRobotArm, scsParameters);
      scsRobotArm.setController(armController);
      scs.getRootRegistry().addChild(registry);
      yoGraphicsListRegistry.registerYoGraphicsList(forcePointController.getYoGraphicsList());
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      /*
      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 1500.0);
      scs.startOnAThread();
      testHelper.setSCSAndCreateSimulationRunner(scs);
      ThreadTools.sleep(TimeTools.secondsToMilliSeconds(1));
      */
      scs.startOnAThread();
      scs.simulate();

      //simulationTestingParameters.setKeepSCSUp(false);
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp() && hasSCSSimulation)
      {
         ThreadTools.sleepForever();
      }
   }

   private static class ForcePointController implements RobotController
   {
      private static final double linearKp = 5.0;
      private static final double linearKi = 1.0;
      private static final double linearKd = 1.0;
      private static final double angularKp = 5.0;
      private static final double angularKi = 1.0;
      private static final double angularKd = 1.0;
      private static final double linearMaxIntegral = 20.0;
      private static final double angularMaxIntegral = 10.0;

      private final YoVariableRegistry registry = new YoVariableRegistry("forcePointController");

      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      private final PIDController pidControllerLinearX;
      private final PIDController pidControllerLinearY;
      private final PIDController pidControllerLinearZ;
      private final PIDController pidControllerX;
      private final PIDController pidControllerY;
      private final PIDController pidControllerZ;

      private final ExternalForcePoint forcePoint;

      private final DoubleYoVariable desiredLinearX = new DoubleYoVariable("desiredLinearX", registry);
      private final DoubleYoVariable desiredLinearY = new DoubleYoVariable("desiredLinearY", registry);
      private final DoubleYoVariable desiredLinearZ = new DoubleYoVariable("desiredLinearZ", registry);
      private final DoubleYoVariable currentLinearX = new DoubleYoVariable("currentLinearX", registry);
      private final DoubleYoVariable currentLinearY = new DoubleYoVariable("currentLinearY", registry);
      private final DoubleYoVariable currentLinearZ = new DoubleYoVariable("currentLinearZ", registry);

      private final DoubleYoVariable desiredAngularX = new DoubleYoVariable("desiredAngularX", registry);
      private final DoubleYoVariable desiredAngularY = new DoubleYoVariable("desiredAngularY", registry);
      private final DoubleYoVariable desiredAngularZ = new DoubleYoVariable("desiredAngularZ", registry);
      private final DoubleYoVariable currentAngularX = new DoubleYoVariable("currentAngularX", registry);
      private final DoubleYoVariable currentAngularY = new DoubleYoVariable("currentAngularY", registry);
      private final DoubleYoVariable currentAngularZ = new DoubleYoVariable("currentAngularZ", registry);

      private final ReferenceFrame handFrame;
      private final FramePose currentPose = new FramePose();
      private final Vector3d desiredPosition = new Vector3d();
      private final Vector3d currentPosition = new Vector3d();
      private final Quat4d desiredOrientation = new Quat4d();
      private final Quat4d currentOrientation = new Quat4d();
      private final Vector3d initialForce = new Vector3d();

      private final YoFrameVector contactForce = new YoFrameVector("contactForce", worldFrame, registry);

      private final YoGraphicVector forceVisualizer;
      private final YoGraphicsList yoGraphicsList = new YoGraphicsList("forceGraphicsList");

      private boolean hasInitialForce = false;

      public ForcePointController(ExternalForcePoint forcePoint, ReferenceFrame handFrame, FramePose desiredPose)
      {
         this.forcePoint = forcePoint;
         this.handFrame = handFrame;
         this.currentPose.setToZero(handFrame);
         desiredPose.getPosition(desiredPosition);
         desiredPose.getOrientation(desiredOrientation);

         YoPIDGains linearPidGains = new YoPIDGains(forcePoint.getName() + "_linear_", registry);
         linearPidGains.setKp(linearKp);
         linearPidGains.setKi(linearKi);
         linearPidGains.setKd(linearKd);
         linearPidGains.setMaximumIntegralError(linearMaxIntegral);

         YoPIDGains angularPidGains = new YoPIDGains(forcePoint.getName() + "_angular_", registry);
         angularPidGains.setKp(angularKp);
         angularPidGains.setKi(angularKi);
         angularPidGains.setKd(angularKd);
         angularPidGains.setMaximumIntegralError(angularMaxIntegral);

         pidControllerX = new PIDController(angularPidGains, "linear_X", registry);
         pidControllerY = new PIDController(angularPidGains, "linear_Y", registry);
         pidControllerZ = new PIDController(angularPidGains, "linear_Z", registry);
         pidControllerLinearX = new PIDController(linearPidGains, "angular_X", registry);
         pidControllerLinearY = new PIDController(linearPidGains, "angular_Y", registry);
         pidControllerLinearZ = new PIDController(linearPidGains, "angular_Z", registry);


         AppearanceDefinition forceAppearance = YoAppearance.Red();
         forcePoint.getYoPosition().getFramePointCopy();
         forceVisualizer = new YoGraphicVector("contactForceVisualizer", forcePoint.getYoPosition(), contactForce, 0.05, forceAppearance);

         yoGraphicsList.add(forceVisualizer);
      }

      public YoGraphicsList getYoGraphicsList()
      {
         return yoGraphicsList;
      }

      public void setInitialForce(Vector3d initialForce)
      {
         forcePoint.setForce(initialForce);
         this.initialForce.set(initialForce);
         hasInitialForce = true;
      }

      public void initialize()
      {
         if (!hasInitialForce)
            forcePoint.setForce(0.0, 0.0, 0.0);

         pidControllerX.resetIntegrator();
         pidControllerY.resetIntegrator();
         pidControllerZ.resetIntegrator();
         pidControllerLinearX.resetIntegrator();
         pidControllerLinearY.resetIntegrator();
         pidControllerLinearZ.resetIntegrator();
      }

      public void doControl()
      {
         currentPose.setToZero(handFrame);
         currentPose.changeFrame(worldFrame);
         currentPose.getPosition(currentPosition);
         currentPose.getOrientation(currentOrientation);

         desiredLinearX.set(desiredPosition.getX());
         desiredLinearY.set(desiredPosition.getY());
         desiredLinearZ.set(desiredPosition.getZ());
         desiredAngularX.set(desiredOrientation.getX());
         desiredAngularY.set(desiredOrientation.getY());
         desiredAngularZ.set(desiredOrientation.getZ());

         currentLinearX.set(currentPosition.getX());
         currentLinearY.set(currentPosition.getY());
         currentLinearZ.set(currentPosition.getZ());
         currentAngularX.set(currentOrientation.getX());
         currentAngularY.set(currentOrientation.getY());
         currentAngularZ.set(currentOrientation.getZ());

         double linearForceX = pidControllerLinearX.compute(currentPosition.getX(), desiredPosition.getX(), 0.0, 0.0, 0.0);
         double linearForceY = pidControllerLinearY.compute(currentPosition.getY(), desiredPosition.getY(), 0.0, 0.0, 0.0);
         double linearForceZ = pidControllerLinearZ.compute(currentPosition.getZ(), desiredPosition.getZ(), 0.0, 0.0, 0.0);

         linearForceX += initialForce.getX();
         linearForceY += initialForce.getY();
         linearForceZ += initialForce.getZ();

         contactForce.setX(linearForceX);
         contactForce.setY(linearForceY);
         contactForce.setZ(linearForceZ);

         double torqueX = pidControllerX.compute(currentOrientation.getX(), desiredOrientation.getX(), 0.0, 0.0, 0.0);
         double torqueY = pidControllerY.compute(currentOrientation.getY(), desiredOrientation.getY(), 0.0, 0.0, 0.0);
         double torqueZ = pidControllerZ.compute(currentOrientation.getZ(), desiredOrientation.getZ(), 0.0, 0.0, 0.0);

         forcePoint.setForce(linearForceX, linearForceY, linearForceZ);
         forcePoint.setMoment(torqueX, torqueY, torqueZ);

         forceVisualizer.update();
      }

      public String getName()
      {
         return "robotArmController";
      }

      public String getDescription()
      {
         return getName();
      }

      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }
   }

   private static class DummyArmController implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry("controller");

      private final SCSRobotFromInverseDynamicsRobotModel scsRobot;
      private final FullRobotModel controllerModel;
      private final OneDoFJoint[] controlledJoints;

      private final ForcePointController forcePointController;

      private final VirtualModelController virtualModelController;
      private final YoWrench yoDesiredWrench;

      private Wrench desiredWrench = new Wrench();

      private final RigidBody hand;

      private boolean firstTick = true;

      public DummyArmController(SCSRobotFromInverseDynamicsRobotModel scsRobot, FullRobotModel controllerModel, OneDoFJoint[] controlledJoints,
            ForcePointController forcePointController, VirtualModelController virtualModelController, RigidBody hand, YoWrench yoDesiredWrench)
      {
         this.scsRobot = scsRobot;
         this.controllerModel = controllerModel;
         this.controlledJoints = controlledJoints;
         this.forcePointController = forcePointController;
         this.virtualModelController = virtualModelController;
         this.hand = hand;
         this.yoDesiredWrench = yoDesiredWrench;

         registry.addChild(forcePointController.getYoVariableRegistry());
      }

      public void initialize()
      {
         forcePointController.initialize();
      }

      public void doControl()
      {
         // copy from scs
         scsRobot.updateJointPositions_SCS_to_ID();
         scsRobot.updateJointVelocities_SCS_to_ID();
         scsRobot.update();
         controllerModel.updateFrames();

         forcePointController.doControl();

         // compute forces
         VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();
         desiredWrench = yoDesiredWrench.getWrench();
         virtualModelController.submitEndEffectorVirtualWrench(hand, desiredWrench);
         virtualModelController.compute(virtualModelControlSolution);

         Map<InverseDynamicsJoint, Double> jointTorques = virtualModelControlSolution.getJointTorques();
         for (OneDoFJoint joint : controlledJoints)
         {
            joint.setTau(jointTorques.get(joint));
         }

         // write to scs
         scsRobot.updateJointPositions_ID_to_SCS();
         scsRobot.updateJointVelocities_ID_to_SCS();
         scsRobot.updateJointTorques_ID_to_SCS();
      }

      public String getName()
      {
         return "robotArmController";
      }

      public String getDescription()
      {
         return getName();
      }

      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }
   }


   private void submitAndCheckVMC(RigidBody base, RigidBody endEffector, Wrench desiredWrench, DenseMatrix64F selectionMatrix)
   {
      simulationTestingParameters.setKeepSCSUp(false);

      InverseDynamicsJoint[] controlledJoints = ScrewTools.createJointPath(base, endEffector);
      GeometricJacobian jacobian = new GeometricJacobian(controlledJoints, base.getBodyFixedFrame());
      jacobian.compute();

      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      DenseMatrix64F transposeJacobianMatrix = new DenseMatrix64F(jacobianMatrix.numCols, Wrench.SIZE);
      CommonOps.transpose(jacobianMatrix, transposeJacobianMatrix);
      CommonOps.invert(transposeJacobianMatrix);

      VirtualModelController virtualModelController = new VirtualModelController(new GeometricJacobianHolder(), base, null, null);
      virtualModelController.registerEndEffector(base, endEffector);

      desiredWrench.changeFrame(base.getBodyFixedFrame());

      if (selectionMatrix == null)
         virtualModelController.submitEndEffectorVirtualWrench(endEffector, desiredWrench);
      else
         virtualModelController.submitEndEffectorVirtualWrench(endEffector, desiredWrench, selectionMatrix);

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

   public static void main(String[] args) throws Exception
   {
      testVMCWithPlanarArm();
   }
}
