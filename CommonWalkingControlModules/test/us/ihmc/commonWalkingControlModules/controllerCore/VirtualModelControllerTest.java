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
import us.ihmc.robotics.robotSide.SideDependentList;
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
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.thread.ThreadTools;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;

//@DeployableTestClass(targets = TestPlanTarget.Fast)
public class VirtualModelControllerTest
{
   private final Random bigRandom = new Random(1000L);
   private final Random random = new Random();
   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private boolean hasSCSSimulation = false;

   /*
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
      simulationTestingParameters.setKeepSCSUp(true);
      hasSCSSimulation = true;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      VirtualModelControllerTestHelper.RobotArm robotArm = testHelper.createRobotArm();
      SCSRobotFromInverseDynamicsRobotModel scsRobotArm = robotArm.getSCSRobotArm();

      List<RigidBody> endEffectors = new ArrayList<>();
      RigidBody hand = robotArm.getHand();
      ReferenceFrame handFrame = robotArm.getHandFrame();
      endEffectors.add(hand);

      FramePose desiredHandPose = new FramePose(handFrame);
      desiredHandPose.setToZero();
      desiredHandPose.changeFrame(worldFrame);

      GeometricJacobianHolder geometricJacobianHolder = new GeometricJacobianHolder();
      VirtualModelController virtualModelController = new VirtualModelController(geometricJacobianHolder, robotArm.getElevator(), registry, yoGraphicsListRegistry);
      virtualModelController.registerEndEffector(robotArm.getElevator(), hand);

      double forceX = random.nextDouble() * 10.0;
      double forceY = random.nextDouble() * 10.0;
      double forceZ = random.nextDouble() * 10.0;
      double torqueX = random.nextDouble() * 10.0;
      double torqueY = random.nextDouble() * 10.0;
      double torqueZ = random.nextDouble() * 10.0;
      Wrench desiredWrench = new Wrench(handFrame, handFrame);
      Vector3d desiredForce = new Vector3d(forceX, forceY, forceZ);
      Vector3d desiredTorque = new Vector3d(torqueX, torqueY, torqueZ);
      FrameVector forceFrameVector = new FrameVector(worldFrame, desiredForce);
      FrameVector torqueFrameVector = new FrameVector(worldFrame, desiredTorque);
      forceFrameVector.changeFrame(handFrame);
      torqueFrameVector.changeFrame(handFrame);
      desiredWrench.set(forceFrameVector, torqueFrameVector);
      desiredWrench.changeFrame(worldFrame);

      List<YoWrench> desiredWrenches = new ArrayList<>();
      YoWrench yoDesiredWrench = new YoWrench("desiredWrench", handFrame, ReferenceFrame.getWorldFrame(), registry);
      yoDesiredWrench.set(desiredWrench);
      desiredWrenches.add(yoDesiredWrench);

      Vector3d contactForce = new Vector3d();
      Vector3d contactTorque = new Vector3d();
      contactForce.set(desiredForce);
      contactTorque.set(desiredTorque);
      contactForce.scale(-1.0);
      contactTorque.scale(-1.0);

      List<ForcePointController> forcePointControllers = new ArrayList<>();
      ForcePointController forcePointController = new ForcePointController(robotArm.getExternalForcePoint(), handFrame, desiredHandPose);
      forcePointController.setInitialForce(contactForce, contactTorque);
      forcePointController.setAngularGains(0.0, 0.0, 5.0);
      forcePointControllers.add(forcePointController);

      DummyArmController armController = new DummyArmController(scsRobotArm, robotArm, robotArm.getOneDoFJoints(), forcePointControllers, virtualModelController,
            geometricJacobianHolder, endEffectors, desiredWrenches);

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters();
      SimulationConstructionSet scs = new SimulationConstructionSet(scsRobotArm, scsParameters);
      scsRobotArm.setController(armController);
      scs.getRootRegistry().addChild(registry);
      yoGraphicsListRegistry.registerYoGraphicsList(forcePointController.getYoGraphicsList());
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 1500.0);
      scs.startOnAThread();

      Vector3d currentPosition = new Vector3d();
      Quat4d currentOrientation = new Quat4d();
      Vector3d currentForce = new Vector3d();
      Vector3d currentTorque = new Vector3d();
      Vector3d desiredPosition = armController.getDesiredPosition(0);
      Quat4d desiredOrientation = armController.getDesiredOrientation(0);

      // check that the end effector doesn't move, and that the desired force is very close to what we want
      double timeIncrement = 1.0;
      while (scs.getTime() < simulationDuration)
      {
         blockingSimulationRunner.simulateAndBlock(timeIncrement);
         currentPosition.set(armController.getCurrentPosition(0));
         currentOrientation.set(armController.getCurrentOrientation(0));
         currentForce.set(armController.getCurrentForce(0));
         currentTorque.set(armController.getCurrentTorque(0));

         JUnitTools.assertVector3dEquals("", currentPosition, desiredPosition, 0.05);
         JUnitTools.assertQuaternionsEqual(currentOrientation, desiredOrientation, 0.05);
         JUnitTools.assertVector3dEquals("", currentTorque, desiredTorque, 0.5);
         JUnitTools.assertVector3dEquals("", currentForce, desiredForce, 0.5);
      }

      simulationTestingParameters.setKeepSCSUp(false);
   }

   @DeployableTestMethod
   @Test(timeout = 3000000)
   public void testVMCWithPlanarArm() throws Exception
   {
      double simulationDuration = 20.0;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      simulationTestingParameters.setKeepSCSUp(true);
      hasSCSSimulation = true;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      VirtualModelControllerTestHelper.PlanarRobotArm robotArm = testHelper.createPlanarArm();
      SCSRobotFromInverseDynamicsRobotModel scsRobotArm = robotArm.getSCSRobotArm();

      List<RigidBody> endEffectors = new ArrayList<>();
      RigidBody hand = robotArm.getHand();
      ReferenceFrame handFrame = robotArm.getHandFrame();
      endEffectors.add(hand);

      FramePose desiredHandPose = new FramePose(handFrame);
      desiredHandPose.setToZero();
      desiredHandPose.changeFrame(worldFrame);

      GeometricJacobianHolder geometricJacobianHolder = new GeometricJacobianHolder();
      VirtualModelController virtualModelController = new VirtualModelController(geometricJacobianHolder, robotArm.getElevator(), registry, yoGraphicsListRegistry);
      virtualModelController.registerEndEffector(robotArm.getElevator(), hand);

      double forceX = random.nextDouble() * 10.0;
      double forceZ = random.nextDouble() * 10.0;
      double torqueY = random.nextDouble() * 10.0;
      Wrench desiredWrench = new Wrench(handFrame, handFrame);
      Vector3d desiredForce = new Vector3d(forceX, 0.0, forceZ);
      Vector3d desiredTorque = new Vector3d(0.0, torqueY, 0.0);
      FrameVector forceFrameVector = new FrameVector(worldFrame, desiredForce);
      FrameVector torqueFrameVector = new FrameVector(worldFrame, desiredTorque);
      forceFrameVector.changeFrame(handFrame);
      torqueFrameVector.changeFrame(handFrame);
      desiredWrench.set(forceFrameVector, torqueFrameVector);
      desiredWrench.changeFrame(worldFrame);

      List<YoWrench> desiredWrenches = new ArrayList<>();
      YoWrench yoDesiredWrench = new YoWrench("desiredWrench", handFrame, ReferenceFrame.getWorldFrame(), registry);
      yoDesiredWrench.set(desiredWrench);
      desiredWrenches.add(yoDesiredWrench);

      Vector3d contactForce = new Vector3d();
      Vector3d contactTorque = new Vector3d();
      contactForce.set(desiredForce);
      contactTorque.set(desiredTorque);
      contactForce.scale(-1.0);
      contactTorque.scale(-1.0);

      List<ForcePointController> forcePointControllers = new ArrayList<>();
      ForcePointController forcePointController = new ForcePointController(robotArm.getExternalForcePoint(), handFrame, desiredHandPose);
      forcePointController.setInitialForce(contactForce, contactTorque);
      forcePointControllers.add(forcePointController);

      DummyArmController armController = new DummyArmController(scsRobotArm, robotArm, robotArm.getOneDoFJoints(), forcePointControllers, virtualModelController,
            geometricJacobianHolder, endEffectors, desiredWrenches);

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters();
      //scsParameters.setDataBufferSize((int)1e5);
      SimulationConstructionSet scs = new SimulationConstructionSet(scsRobotArm, scsParameters);
      scsRobotArm.setController(armController);
      scs.getRootRegistry().addChild(registry);
      yoGraphicsListRegistry.registerYoGraphicsList(forcePointController.getYoGraphicsList());
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 1500.0);
      scs.startOnAThread();

      Vector3d currentPosition = new Vector3d();
      Quat4d currentOrientation = new Quat4d();
      Vector3d currentForce = new Vector3d();
      Vector3d currentTorque = new Vector3d();
      Vector3d desiredPosition = armController.getDesiredPosition(0);
      Quat4d desiredOrientation = armController.getDesiredOrientation(0);

      // check that the end effector doesn't move, and that the desired force is very close to what we want
      double timeIncrement = 1.0;
      while (scs.getTime() < simulationDuration)
      {
         blockingSimulationRunner.simulateAndBlock(timeIncrement);
         currentPosition.set(armController.getCurrentPosition(0));
         currentOrientation.set(armController.getCurrentOrientation(0));
         currentForce.set(armController.getCurrentForce(0));
         currentTorque.set(armController.getCurrentTorque(0));

         JUnitTools.assertVector3dEquals("", currentPosition, desiredPosition, 0.01);
         JUnitTools.assertQuaternionsEqual(currentOrientation, desiredOrientation, 0.01);
         JUnitTools.assertVector3dEquals("", desiredForce, currentForce, 0.5);
         JUnitTools.assertVector3dEquals("", desiredTorque, currentTorque, 0.5);
      }

      simulationTestingParameters.setKeepSCSUp(false);
   }
   */

   public static void testVMCWithForkedArm() throws Exception
   {
      double simulationDuration = 20.0;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      Random random = new Random();
      //simulationTestingParameters.setKeepSCSUp(true);
      //hasSCSSimulation = true;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      VirtualModelControllerTestHelper.ForkedRobotArm robotArm = testHelper.createForkedRobotArm();
      SCSRobotFromInverseDynamicsRobotModel scsRobotArm = robotArm.getSCSRobotArm();

      GeometricJacobianHolder geometricJacobianHolder = new GeometricJacobianHolder();
      VirtualModelController virtualModelController = new VirtualModelController(geometricJacobianHolder, robotArm.getElevator(), registry, yoGraphicsListRegistry);

      SideDependentList<RigidBody> hands = robotArm.getHands();
      SideDependentList<ReferenceFrame> handFrames = new SideDependentList<>();
      SideDependentList<FramePose> desiredHandPoses = new SideDependentList<>();

      List<Vector3d> desiredForces = new ArrayList<>();
      List<Vector3d> desiredTorques = new ArrayList<>();
      ArrayList<YoWrench> desiredHandWrenches = new ArrayList<>();
      ArrayList<ForcePointController> forcePointControllers = new ArrayList<>();
      ArrayList<RigidBody> endEffectors = new ArrayList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = hands.get(robotSide);
         ReferenceFrame handFrame = hand.getBodyFixedFrame();
         endEffectors.add(hand);

         FramePose desiredHandPose = new FramePose(handFrame);
         desiredHandPose.setToZero();
         desiredHandPose.changeFrame(worldFrame);

         handFrames.put(robotSide, handFrame);
         desiredHandPoses.put(robotSide, desiredHandPose);

         virtualModelController.registerEndEffector(robotArm.getElevator(), hand);

         double forceX = random.nextDouble() * 10.0;
         double forceZ = random.nextDouble() * 10.0;
         double torqueY = random.nextDouble() * 10.0;
         Wrench desiredWrench = new Wrench(handFrame, handFrame);
         Vector3d desiredForce = new Vector3d(forceX, 0.0, forceZ);
         Vector3d desiredTorque = new Vector3d(0.0, torqueY, 0.0);
         FrameVector forceFrameVector = new FrameVector(worldFrame, desiredForce);
         FrameVector torqueFrameVector = new FrameVector(worldFrame, desiredTorque);
         forceFrameVector.changeFrame(handFrame);
         torqueFrameVector.changeFrame(handFrame);
         desiredWrench.set(forceFrameVector, torqueFrameVector);
         desiredWrench.changeFrame(worldFrame);

         desiredForces.add(desiredForce);
         desiredTorques.add(desiredTorque);

         YoWrench yoDesiredWrench = new YoWrench("desiredWrench" + robotSide.getShortLowerCaseName(), handFrame, ReferenceFrame.getWorldFrame(), registry);
         yoDesiredWrench.set(desiredWrench);

         desiredHandWrenches.add(yoDesiredWrench);

         Vector3d contactForce = new Vector3d();
         Vector3d contactTorque = new Vector3d();
         contactForce.set(desiredForce);
         contactTorque.set(desiredTorque);
         contactForce.scale(-1.0);
         contactTorque.scale(-1.0);

         String suffix;
         if (robotSide == RobotSide.LEFT)
            suffix = "1";
         else
            suffix = "2";

         ForcePointController forcePointController = new ForcePointController(suffix, robotArm.getExternalForcePoints().get(robotSide), handFrame, desiredHandPose);
         forcePointController.setInitialForce(contactForce, contactTorque);
         forcePointControllers.add(forcePointController);
      }

      DummyArmController armController = new DummyArmController(scsRobotArm, robotArm, robotArm.getOneDoFJoints(), forcePointControllers, virtualModelController,
            geometricJacobianHolder, endEffectors, desiredHandWrenches);

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters();
      //scsParameters.setDataBufferSize((int)1e5);
      SimulationConstructionSet scs = new SimulationConstructionSet(scsRobotArm, scsParameters);
      scsRobotArm.setController(armController);
      scs.getRootRegistry().addChild(registry);
      for (ForcePointController forcePointController : forcePointControllers)
         yoGraphicsListRegistry.registerYoGraphicsList(forcePointController.getYoGraphicsList());
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 1500.0);
      scs.startOnAThread();

      Vector3d currentPosition = new Vector3d();
      Quat4d currentOrientation = new Quat4d();
      Vector3d currentForce = new Vector3d();
      Vector3d currentTorque = new Vector3d();

      List<Vector3d> desiredPositions = new ArrayList<>();
      List<Quat4d> desiredOrientations = new ArrayList<>();

      for (int i = 0; i < endEffectors.size(); i++)
      {
         desiredPositions.add(armController.getDesiredPosition(i));
         desiredOrientations.add(armController.getDesiredOrientation(i));
      }

      // check that the end effector doesn't move, and that the desired force is very close to what we want
      double timeIncrement = 1.0;
      while (scs.getTime() < simulationDuration)
      {
         blockingSimulationRunner.simulateAndBlock(timeIncrement);
         for (int i = 0; i < endEffectors.size(); i++)
         {
            currentPosition.set(armController.getCurrentPosition(i));
            currentOrientation.set(armController.getCurrentOrientation(i));
            currentForce.set(armController.getCurrentForce(i));
            currentTorque.set(armController.getCurrentTorque(i));

            JUnitTools.assertVector3dEquals("", currentPosition, desiredPositions.get(i), 0.01);
            JUnitTools.assertQuaternionsEqual(currentOrientation, desiredOrientations.get(i), 0.01);
            JUnitTools.assertVector3dEquals("", desiredForces.get(i), currentForce, 0.5);
            JUnitTools.assertVector3dEquals("", desiredTorques.get(i), currentTorque, 0.5);
         }
      }

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
      private static final double linearKp = 50.0;
      private static final double linearKi = 0.0;
      private static final double linearKd = 50.0;
      private static final double linearDeadband = 0.00;
      private static final double angularKp = 0.0;
      private static final double angularKi = 0.0;
      private static final double angularKd = 10.0;
      private static final double angularDeadband = 0.00;
      private static final double linearMaxIntegral = 50.0;
      private static final double angularMaxIntegral = 50.0;

      private final YoVariableRegistry registry;

      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      private final YoPIDGains linearPidGains;
      private final YoPIDGains angularPidGains;

      private final PIDController pidControllerLinearX;
      private final PIDController pidControllerLinearY;
      private final PIDController pidControllerLinearZ;
      private final PIDController pidControllerAngularX;
      private final PIDController pidControllerAngularY;
      private final PIDController pidControllerAngularZ;

      private final ExternalForcePoint forcePoint;

      private final DoubleYoVariable desiredLinearX;
      private final DoubleYoVariable desiredLinearY;
      private final DoubleYoVariable desiredLinearZ;
      private final DoubleYoVariable currentLinearX;
      private final DoubleYoVariable currentLinearY;
      private final DoubleYoVariable currentLinearZ;

      private final DoubleYoVariable desiredAngularX;
      private final DoubleYoVariable desiredAngularY;
      private final DoubleYoVariable desiredAngularZ;
      private final DoubleYoVariable currentAngularX;
      private final DoubleYoVariable currentAngularY;
      private final DoubleYoVariable currentAngularZ;

      private final ReferenceFrame handFrame;
      private final FramePose currentPose = new FramePose();
      private final Vector3d desiredPosition = new Vector3d();
      private final Vector3d currentPosition = new Vector3d();
      private final Quat4d desiredOrientation = new Quat4d();
      private final Quat4d currentOrientation = new Quat4d();
      private final Vector3d initialForce = new Vector3d();
      private final Vector3d initialTorque = new Vector3d();

      private final YoFrameVector contactForce;
      private final YoFrameVector contactTorque;

      private final YoGraphicVector forceVisualizer;
      private final YoGraphicsList yoGraphicsList;

      private boolean hasInitialForce = false;

      public ForcePointController(ExternalForcePoint forcePoint, ReferenceFrame handFrame, FramePose desiredPose)
      {
         this("", forcePoint, handFrame, desiredPose);
      }

      public ForcePointController(String suffix, ExternalForcePoint forcePoint, ReferenceFrame handFrame, FramePose desiredPose)
      {
         this.forcePoint = forcePoint;
         this.handFrame = handFrame;
         this.currentPose.setToZero(handFrame);
         desiredPose.getPosition(desiredPosition);
         desiredPose.getOrientation(desiredOrientation);

         registry = new YoVariableRegistry("forcePointController" + suffix);

         desiredLinearX = new DoubleYoVariable("desiredLinearX" + suffix, registry);
         desiredLinearY = new DoubleYoVariable("desiredLinearY" + suffix, registry);
         desiredLinearZ = new DoubleYoVariable("desiredLinearZ" + suffix, registry);
         currentLinearX = new DoubleYoVariable("currentLinearX" + suffix, registry);
         currentLinearY = new DoubleYoVariable("currentLinearY" + suffix, registry);
         currentLinearZ = new DoubleYoVariable("currentLinearZ" + suffix, registry);

         desiredAngularX = new DoubleYoVariable("desiredAngularX" + suffix, registry);
         desiredAngularY = new DoubleYoVariable("desiredAngularY" + suffix, registry);
         desiredAngularZ = new DoubleYoVariable("desiredAngularZ" + suffix, registry);
         currentAngularX = new DoubleYoVariable("currentAngularX" + suffix, registry);
         currentAngularY = new DoubleYoVariable("currentAngularY" + suffix, registry);
         currentAngularZ = new DoubleYoVariable("currentAngularZ" + suffix, registry);

         contactForce = new YoFrameVector("contactForce" + suffix, worldFrame, registry);
         contactTorque = new YoFrameVector("contactTorque" + suffix, worldFrame, registry);

         yoGraphicsList = new YoGraphicsList("forceGraphicsList" + suffix);

         linearPidGains = new YoPIDGains(forcePoint.getName() + "_linear" + suffix, registry);
         linearPidGains.setKp(linearKp);
         linearPidGains.setKi(linearKi);
         linearPidGains.setKd(linearKd);
         linearPidGains.setMaximumIntegralError(linearMaxIntegral);
         linearPidGains.setPositionDeadband(linearDeadband);

         angularPidGains = new YoPIDGains(forcePoint.getName() + "_angular" + suffix, registry);
         angularPidGains.setKp(angularKp);
         angularPidGains.setKi(angularKi);
         angularPidGains.setKd(angularKd);
         angularPidGains.setMaximumIntegralError(angularMaxIntegral);
         angularPidGains.setPositionDeadband(angularDeadband);

         pidControllerAngularX = new PIDController(angularPidGains, "angular_X" + suffix, registry);
         pidControllerAngularY = new PIDController(angularPidGains, "angular_Y" + suffix, registry);
         pidControllerAngularZ = new PIDController(angularPidGains, "angular_Z" + suffix, registry);

         pidControllerLinearX = new PIDController(linearPidGains, "linear_X" + suffix, registry);
         pidControllerLinearY = new PIDController(linearPidGains, "linear_Y" + suffix, registry);
         pidControllerLinearZ = new PIDController(linearPidGains, "linear_Z" + suffix, registry);

         AppearanceDefinition forceAppearance = YoAppearance.Red();
         forcePoint.getYoPosition().getFramePointCopy();
         forceVisualizer = new YoGraphicVector("contactForceVisualizer" + suffix, forcePoint.getYoPosition(), contactForce, 0.05, forceAppearance);

         yoGraphicsList.add(forceVisualizer);
      }

      public YoGraphicsList getYoGraphicsList()
      {
         return yoGraphicsList;
      }

      public void setLinearGains(double kp, double ki, double kd)
      {
         linearPidGains.setKp(kp);
         linearPidGains.setKi(ki);
         linearPidGains.setKd(kd);
      }

      public void setAngularGains(double kp, double ki, double kd)
      {
         angularPidGains.setKp(kp);
         angularPidGains.setKi(ki);
         angularPidGains.setKd(kd);
      }

      public void setInitialForce(Vector3d initialForce, Vector3d initialTorque)
      {
         forcePoint.setForce(initialForce);
         forcePoint.setMoment(initialTorque);
         this.initialForce.set(initialForce);
         this.initialTorque.set(initialTorque);
         hasInitialForce = true;
      }

      public void initialize()
      {
         if (!hasInitialForce)
            forcePoint.setForce(0.0, 0.0, 0.0);

         pidControllerAngularX.resetIntegrator();
         pidControllerAngularY.resetIntegrator();
         pidControllerAngularZ.resetIntegrator();

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

         double torqueX = pidControllerAngularX.computeForAngles(currentOrientation.getX(), desiredOrientation.getX(), 0.0, 0.0, 0.0);
         double torqueY = pidControllerAngularY.computeForAngles(currentOrientation.getY(), desiredOrientation.getY(), 0.0, 0.0, 0.0);
         double torqueZ = pidControllerAngularZ.computeForAngles(currentOrientation.getZ(), desiredOrientation.getZ(), 0.0, 0.0, 0.0);

         torqueX += initialTorque.getX();
         torqueY += initialTorque.getY();
         torqueZ += initialTorque.getZ();

         contactTorque.setX(torqueX);
         contactTorque.setY(torqueY);
         contactTorque.setZ(torqueZ);

         forcePoint.setForce(linearForceX, linearForceY, linearForceZ);
         forcePoint.setMoment(torqueX, torqueY, torqueZ);

         forceVisualizer.update();
      }

      public Vector3d getDesiredPosition()
      {
         return desiredPosition;
      }

      public Quat4d getDesiredOrientation()
      {
         return desiredOrientation;
      }

      public Vector3d getCurrentPosition()
      {
         return currentPosition;
      }

      public Vector3d getCurrentTorque()
      {
         Vector3d contactTorque = new Vector3d();
         this.contactTorque.get(contactTorque);
         contactTorque.negate();
         return contactTorque;
      }

      public Vector3d getCurrentForce()
      {
         Vector3d contactForce = new Vector3d();
         this.contactForce.get(contactForce);
         contactForce.negate();
         return contactForce;
      }

      public Quat4d getCurrentOrientation()
      {
         return currentOrientation;
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

      private final DoubleYoVariable shoulderSolution = new DoubleYoVariable("shoulderTorque", registry);
      private final DoubleYoVariable elbowSolution = new DoubleYoVariable("elbowTorque", registry);
      private final DoubleYoVariable wristSolution = new DoubleYoVariable("wristTorque", registry);

      private final SCSRobotFromInverseDynamicsRobotModel scsRobot;
      private final FullRobotModel controllerModel;
      private final OneDoFJoint[] controlledJoints;

      private final VirtualModelController virtualModelController;
      private final GeometricJacobianHolder geometricJacobianHolder;

      private Wrench desiredWrench = new Wrench();

      private List<ForcePointController> forcePointControllers = new ArrayList<>();
      private List<YoWrench> yoDesiredWrenches = new ArrayList<>();
      private List<RigidBody> endEffectors = new ArrayList<>();

      private boolean firstTick = true;

      public DummyArmController(SCSRobotFromInverseDynamicsRobotModel scsRobot, FullRobotModel controllerModel, OneDoFJoint[] controlledJoints,
            List<ForcePointController> forcePointControllers, VirtualModelController virtualModelController, GeometricJacobianHolder geometricJacobianHolder,
            List<RigidBody> endEffectors, List<YoWrench> yoDesiredWrenches)
      {
         this.scsRobot = scsRobot;
         this.controllerModel = controllerModel;
         this.controlledJoints = controlledJoints;
         this.forcePointControllers = forcePointControllers;
         this.virtualModelController = virtualModelController;
         this.geometricJacobianHolder = geometricJacobianHolder;
         this.endEffectors = endEffectors;
         this.yoDesiredWrenches = yoDesiredWrenches;

         for (ForcePointController forcePointController : forcePointControllers)
            registry.addChild(forcePointController.getYoVariableRegistry());
      }

      public void initialize()
      {
         for (ForcePointController forcePointController : forcePointControllers)
            forcePointController.initialize();
      }

      public void doControl()
      {
         // copy from scs
         scsRobot.updateJointPositions_SCS_to_ID();
         scsRobot.updateJointVelocities_SCS_to_ID();
         scsRobot.update();
         controllerModel.updateFrames();

         for (ForcePointController forcePointController : forcePointControllers)
            forcePointController.doControl();
         geometricJacobianHolder.compute();

         // compute forces
         VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();

         for (int i = 0; i < endEffectors.size(); i++)
         {
            desiredWrench.set(yoDesiredWrenches.get(i).getWrench());
            virtualModelController.submitEndEffectorVirtualWrench(endEffectors.get(i), desiredWrench);
         }
         virtualModelController.compute(virtualModelControlSolution);

         Map<InverseDynamicsJoint, Double> jointTorques = virtualModelControlSolution.getJointTorques();
         for (OneDoFJoint joint : controlledJoints)
         {
            if (joint.getName().contains("shoulder"))
               shoulderSolution.set(jointTorques.get(joint));
            else if (joint.getName().contains("elbow"))
               elbowSolution.set(jointTorques.get(joint));
            else if (joint.getName().contains("wrist"))
               wristSolution.set(jointTorques.get(joint));

            joint.setTau(jointTorques.get(joint));
         }

         // write to scs
         scsRobot.updateJointPositions_ID_to_SCS();
         scsRobot.updateJointVelocities_ID_to_SCS();
         scsRobot.updateJointTorques_ID_to_SCS();
      }

      public Vector3d getDesiredPosition(int index)
      {
         return forcePointControllers.get(index).getDesiredPosition();
      }

      public Quat4d getDesiredOrientation(int index)
      {
         return forcePointControllers.get(index).getDesiredOrientation();
      }

      public Vector3d getCurrentPosition(int index)
      {
         return forcePointControllers.get(index).getCurrentPosition();
      }

      public Quat4d getCurrentOrientation(int index)
      {
         return forcePointControllers.get(index).getCurrentOrientation();
      }

      public Vector3d getCurrentForce(int index)
      {
         return forcePointControllers.get(index).getCurrentForce();
      }

      public Vector3d getCurrentTorque(int index)
      {
         return forcePointControllers.get(index).getCurrentTorque();
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
      testVMCWithForkedArm();
   }
}
