package us.ihmc.commonWalkingControlModules.controllerCore;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.VirtualModelControllerTestHelper.RobotLeg;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Vector3d;
import java.util.Map;
import java.util.Random;

public class VirtualModelControllerTest
{
   private final Random bigRandom = new Random(1000L);

   @DeployableTestMethod
   @Test(timeout = 500)
   public void testJacobianCalculation()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench wrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      InverseDynamicsJoint[] controlledJoints = ScrewTools.createJointPath(pelvis, foot);
      GeometricJacobian jacobian = new GeometricJacobian(controlledJoints, pelvis.getBodyFixedFrame());
      jacobian.compute();

      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      DenseMatrix64F transposeJacobianMatrix = new DenseMatrix64F(Wrench.SIZE, Wrench.SIZE);
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
   @Test(timeout = 500)
   public void testVMC()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame and no selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      submitAndCheckVMC(pelvis, foot, desiredWrench, null);
   }

   @DeployableTestMethod
   @Test(timeout = 500)
   public void testVMCSelectAll()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      submitAndCheckVMC(pelvis, foot, desiredWrench, CommonOps.identity(Wrench.SIZE, Wrench.SIZE));
   }

   @DeployableTestMethod
   @Test(timeout = 500)
   public void testVMCSelectForce()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
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
   @Test(timeout = 500)
   public void testVMCSelectTorque()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
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
   @Test(timeout = 500)
   public void testVMCSelectForceX()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
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
   @Test(timeout = 500)
   public void testVMCSelectForceY()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
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
   @Test(timeout = 500)
   public void testVMCSelectForceZ()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
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
   @Test(timeout = 500)
   public void testVMCSelectTorqueX()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
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
   @Test(timeout = 500)
   public void testVMCSelectTorqueY()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
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
   @Test(timeout = 500)
   public void testVMCSelectTorqueZ()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
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
   @Test(timeout = 500)
   public void testVMCSelectForceXTorqueY()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
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
   @Test(timeout = 500)
   public void testVMCSelectForceYZTorqueX()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
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
   @Test(timeout = 500)
   public void testVMCSelectForceXTorqueXZ()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
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
   @Test(timeout = 500)
   public void testVMCWrongExpressedInFrame()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), base.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 2, 1);
      selectionMatrix.set(2, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 500)
   public void testVMCWrongExpressedOnFrame()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(base.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 2, 1);
      selectionMatrix.set(2, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 500)
   public void testVMCWrongExpressedInAndOnFrame()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(base.getBodyFixedFrame(), base.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 2, 1);
      selectionMatrix.set(2, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 500)
   public void testVMCVirtualWrenchCommand()
   {
      double gravity = -9.81;

      VirtualModelControllerTestHelper testHelper = new VirtualModelControllerTestHelper();
      RobotLeg robotLeg = testHelper.createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      FrameVector desiredTorque = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(bigRandom.nextDouble(), bigRandom.nextDouble(), bigRandom.nextDouble()));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), desiredTorque.getVector());

      DenseMatrix64F selectionMatrix = CommonOps.identity(Wrench.SIZE, Wrench.SIZE);

      InverseDynamicsJoint[] controlledJoints = ScrewTools.createJointPath(base, endEffector);
      GeometricJacobian jacobian = new GeometricJacobian(controlledJoints, base.getBodyFixedFrame());
      jacobian.compute();

      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      DenseMatrix64F transposeJacobianMatrix = new DenseMatrix64F(Wrench.SIZE, Wrench.SIZE);
      CommonOps.transpose(jacobianMatrix, transposeJacobianMatrix);
      CommonOps.invert(transposeJacobianMatrix);

      VirtualModelController virtualModelController = new VirtualModelController(new GeometricJacobianHolder(), base);
      virtualModelController.registerEndEffector(base, endEffector);

      VirtualWrenchCommand virtualWrenchCommand = new VirtualWrenchCommand();
      virtualWrenchCommand.set(foot, desiredWrench, selectionMatrix);

      virtualModelController.submitEndEffectorVirtualWrench(virtualWrenchCommand);

      // find jacobian transpose solution
      VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();
      virtualModelController.compute(virtualModelControlSolution);

      desiredWrench.changeFrame(base.getBodyFixedFrame());

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

   private void submitAndCheckVMC(RigidBody base, RigidBody endEffector, Wrench desiredWrench, DenseMatrix64F selectionMatrix)
   {
      InverseDynamicsJoint[] controlledJoints = ScrewTools.createJointPath(base, endEffector);
      GeometricJacobian jacobian = new GeometricJacobian(controlledJoints, base.getBodyFixedFrame());
      jacobian.compute();

      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      DenseMatrix64F transposeJacobianMatrix = new DenseMatrix64F(Wrench.SIZE, Wrench.SIZE);
      CommonOps.transpose(jacobianMatrix, transposeJacobianMatrix);
      CommonOps.invert(transposeJacobianMatrix);

      VirtualModelController virtualModelController = new VirtualModelController(new GeometricJacobianHolder(), base);
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
}
