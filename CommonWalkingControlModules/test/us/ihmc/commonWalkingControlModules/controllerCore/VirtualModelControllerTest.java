package us.ihmc.commonWalkingControlModules.controllerCore;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.Map;
import java.util.Random;

public class VirtualModelControllerTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

   public static final double POUNDS = 1.0 / 2.2;    // Pound to Kg conversion.
   public static final double INCHES = 0.0254;    // Inch to Meter Conversion.

   public static final double PELVIS_HEIGHT = 1.0;
   public static final double PELVIS_RAD = 0.1;

   public static final double HIP_WIDTH = 0.2;

   public static final double HIP_DIFFERENTIAL_HEIGHT = 0.05;
   public static final double HIP_DIFFERENTIAL_WIDTH = 0.075;

   public static final double THIGH_LENGTH = 23.29 * INCHES;
   public static final double THIGH_RAD = 0.05;
   public static final double THIGH_MASS = 6.7 * POUNDS;

   public static final double SHIN_LENGTH = 9.1 * INCHES;
   public static final double SHIN_RAD = 0.03;
   public static final double SHIN_MASS = 1.0 * POUNDS;

   public static final double ANKLE_DIFFERENTIAL_HEIGHT = 0.025;
   public static final double ANKLE_DIFFERENTIAL_WIDTH = 0.0375;

   public static final double FOOT_LENGTH = 0.08;
   public static final double FOOT_COM_OFFSET = 3.0 * INCHES;
   public static final double FOOT_RAD = 0.05;
   public static final double FOOT_MASS = 3.0 * POUNDS;

   private final Random random = new Random(100L);

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testJacobianCalculation()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench wrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

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

      compareWrenches(wrench, appliedWrench);
   }

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testVMC()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame and no selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

      submitAndCheckVMC(pelvis, foot, desiredWrench, null);
   }

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testVMCSelectAll()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

      submitAndCheckVMC(pelvis, foot, desiredWrench, CommonOps.identity(Wrench.SIZE, Wrench.SIZE));
   }

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testVMCSelectForce()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

      // select only force
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 3, 1);
      selectionMatrix.set(1, 4, 1);
      selectionMatrix.set(2, 5, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testVMCSelectTorque()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 1, 1);
      selectionMatrix.set(2, 2, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testVMCSelectForceX()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testVMCSelectForceY()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 4, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testVMCSelectForceZ()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 5, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testVMCSelectTorqueX()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 0, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testVMCSelectTorqueY()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 1, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testVMCSelectTorqueZ()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 6);
      selectionMatrix.set(0, 2, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testVMCSelectForceXTorqueY()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(2, 6);
      selectionMatrix.set(0, 1, 1);
      selectionMatrix.set(1, 4, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testVMCSelectForceYZTorqueX()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 4, 1);
      selectionMatrix.set(2, 5, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
   }

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testVMCSelectForceXTorqueXZ()
   {
      double gravity = -9.81;

      RobotLeg robotLeg = createRobotLeg(gravity);
      RigidBody base = robotLeg.getBase();
      RigidBody endEffector = robotLeg.getEndEffector();
      RigidBody foot = endEffector.getParentJoint().getSuccessor();
      RigidBody pelvis = robotLeg.getRootJoint().getSuccessor();

      // send in the correct frame with identity selection matrix
      FrameVector desiredForce = new FrameVector(foot.getBodyFixedFrame(), new Vector3d(-20.0, 2.0, 60.0));
      Wrench desiredWrench = new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame(), desiredForce.getVector(), new Vector3d());

      // select only torque
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1);
      selectionMatrix.set(1, 2, 1);
      selectionMatrix.set(2, 3, 1);

      submitAndCheckVMC(pelvis, foot, desiredWrench, selectionMatrix);
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
         compareWrenches(desiredWrench, appliedWrench);
      else
         compareWrenches(desiredWrench, appliedWrench, selectionMatrix);
   }


   private RobotLeg createRobotLeg(double gravity)
   {
      Robot robotLeg = new Robot("robotLeg");
      robotLeg.setGravity(gravity);

      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(), new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);

      FloatingJoint floatingJoint = new FloatingJoint("pelvis", new Vector3d(), robotLeg);
      robotLeg.addRootJoint(floatingJoint);
      SixDoFJoint rootJoint = new SixDoFJoint("pelvis", elevator, elevatorFrame);

      Link pelvisLink = pelvis();
      floatingJoint.setLink(pelvisLink);
      RigidBody pelvisBody = copyLinkAsRigidBody(pelvisLink, rootJoint, "pelvis");

      Vector3d hipYawOffset = new Vector3d(0.0, -HIP_WIDTH, 0.0);
      PinJoint hip_yaw = new PinJoint("hip_yaw", hipYawOffset, robotLeg, Axis.Z);
      hip_yaw.setQ(0);

      Link hip_differential = hip_differential();
      hip_yaw.setLink(hip_differential);
      floatingJoint.addJoint(hip_yaw);

      RevoluteJoint hipYaw = ScrewTools.addRevoluteJoint("hip_yaw", pelvisBody, hipYawOffset, Z);
      hipYaw.setQ(hip_yaw.getQ().getDoubleValue());
      RigidBody hipDifferentialBody = copyLinkAsRigidBody(hip_differential, hipYaw, "hip_differential");

      Vector3d hipRollOffset = new Vector3d();
      PinJoint hip_roll = new PinJoint("hip_roll", hipRollOffset, robotLeg, Axis.X);
      hip_roll.setQ(0);

      Link hip_differential2 = hip_differential();
      hip_roll.setLink(hip_differential2);
      hip_yaw.addJoint(hip_roll);

      RevoluteJoint hipRoll = ScrewTools.addRevoluteJoint("hip_roll", hipDifferentialBody, hipRollOffset, X);
      hipRoll.setQ(hip_roll.getQ().getDoubleValue());
      RigidBody hipDifferentialBody2 = copyLinkAsRigidBody(hip_differential2, hipRoll, "hip_differential");

      Vector3d hipPitchOffset = new Vector3d();
      PinJoint hip_pitch = new PinJoint("hip_pitch", hipPitchOffset, robotLeg, Axis.Y);
      hip_pitch.setQ(-0.2);

      Link thigh = thigh();
      hip_pitch.setLink(thigh);
      hip_roll.addJoint(hip_pitch);

      RevoluteJoint hipPitch = ScrewTools.addRevoluteJoint("hip_pitch", hipDifferentialBody2, hipPitchOffset, Y);
      hipPitch.setQ(hip_pitch.getQ().getDoubleValue());
      RigidBody thighBody = copyLinkAsRigidBody(thigh, hipPitch, "thigh");

      Vector3d kneePitchOffset = new Vector3d(0.0, 0.0, -THIGH_LENGTH);
      PinJoint knee_pitch = new PinJoint("knee_pitch", kneePitchOffset, robotLeg, Axis.Y);
      knee_pitch.setQ(0.4);

      Link shin = shin();
      knee_pitch.setLink(shin);
      hip_pitch.addJoint(knee_pitch);

      RevoluteJoint kneePitch = ScrewTools.addRevoluteJoint("knee_pitch", thighBody, kneePitchOffset, Y);
      kneePitch.setQ(knee_pitch.getQ().getDoubleValue());
      RigidBody shinBody = copyLinkAsRigidBody(shin, kneePitch, "shin");

      Vector3d anklePitchOffset = new Vector3d(0.0, 0.0, -SHIN_LENGTH);
      PinJoint ankle_pitch = new PinJoint("ankle_pitch", anklePitchOffset, robotLeg, Axis.Y);
      ankle_pitch.setQ(0.2);

      Link ankle_differential = ankle_differential();
      ankle_pitch.setLink(ankle_differential);
      knee_pitch.addJoint(ankle_pitch);

      RevoluteJoint anklePitch = ScrewTools.addRevoluteJoint("ankle_pitch", shinBody, anklePitchOffset, Y);
      anklePitch.setQ(ankle_pitch.getQ().getDoubleValue());
      RigidBody ankleDifferentialBody = copyLinkAsRigidBody(ankle_differential, anklePitch, "ankle_differential");

      Vector3d ankleRollOffset = new Vector3d();
      PinJoint ankle_roll = new PinJoint("ankle_roll", ankleRollOffset, robotLeg, Axis.X);
      ankle_roll.setQ(0.0);

      Link foot = foot();
      ankle_roll.setLink(foot);
      ankle_pitch.addJoint(ankle_roll);

      RevoluteJoint ankleRoll = ScrewTools.addRevoluteJoint("ankle_roll", ankleDifferentialBody, ankleRollOffset, X);
      ankleRoll.setQ(ankle_roll.getQ().getDoubleValue());
      RigidBody footBody = copyLinkAsRigidBody(foot, ankleRoll, "foot");

      return new RobotLeg(pelvisBody, footBody, rootJoint);
   }

   private Link pelvis()
   {
      AppearanceDefinition pelvisAppearance = YoAppearance.Blue();

      Link ret = new Link("pelvis");

      ret.setMass(100.0);
      ret.setMomentOfInertia(1.0, 1.0, 1.0);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCoordinateSystem(1.0);
      linkGraphics.addCylinder(PELVIS_HEIGHT, PELVIS_RAD, pelvisAppearance);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link hip_differential()
   {
      Link ret = new Link("hip_differential");

      ret.setMass(0.1);
      ret.setMomentOfInertia(0.005, 0.005, 0.005);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCube(HIP_DIFFERENTIAL_WIDTH, HIP_DIFFERENTIAL_WIDTH, HIP_DIFFERENTIAL_HEIGHT);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   private Link thigh()
   {
      AppearanceDefinition thighApp = YoAppearance.Green();

      Link ret = new Link("thigh");

      ret.setMass(THIGH_MASS);    // 2.35);
      ret.setMomentOfInertia(0.0437, 0.0437, 0.0054);
      ret.setComOffset(0.0, 0.0, -THIGH_LENGTH / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(THIGH_LENGTH, THIGH_RAD, thighApp);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   private Link shin()
   {
      AppearanceDefinition shinApp = YoAppearance.Red();

      Link ret = new Link("shin");

      ret.setMass(SHIN_MASS);    // 0.864);
      ret.setMomentOfInertia(0.00429, 0.00429, 0.00106);
      ret.setComOffset(0.0, 0.0, -SHIN_LENGTH / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(SHIN_LENGTH, SHIN_RAD, shinApp);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link ankle_differential()
   {
      Link ret = new Link("ankle_differential");

      ret.setMass(0.1);
      ret.setMomentOfInertia(0.005, 0.005, 0.005);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCube(ANKLE_DIFFERENTIAL_WIDTH, ANKLE_DIFFERENTIAL_WIDTH, ANKLE_DIFFERENTIAL_HEIGHT);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link foot()
   {
      AppearanceDefinition footApp = YoAppearance.PlaneMaterial();

      Link ret = new Link("foot");

      ret.setMass(FOOT_MASS);    // 0.207);
      ret.setMomentOfInertia(0.00041, 0.00041, 0.00001689);
      ret.setComOffset(FOOT_COM_OFFSET, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(FOOT_LENGTH, FOOT_RAD, footApp);

      linkGraphics.translate(0.05, 0.0, FOOT_LENGTH);
      linkGraphics.addCube(0.02, 0.1, 0.1, YoAppearance.Black());

      linkGraphics.translate(-0.1, 0.0, 0.0);
      linkGraphics.addCube(0.02, 0.1, 0.1, YoAppearance.Black());

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }
   private RigidBody copyLinkAsRigidBody(Link link, InverseDynamicsJoint currentInverseDynamicsJoint, String bodyName)
   {
      Vector3d comOffset = new Vector3d();
      link.getComOffset(comOffset);
      Matrix3d momentOfInertia = new Matrix3d();
      link.getMomentOfInertia(momentOfInertia);
      ReferenceFrame nextFrame = createOffsetFrame(currentInverseDynamicsJoint, comOffset, bodyName);
      nextFrame.update();
      RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, link.getMass());
      RigidBody rigidBody = new RigidBody(bodyName, inertia, currentInverseDynamicsJoint);

      return rigidBody;
   }

   private static ReferenceFrame createOffsetFrame(InverseDynamicsJoint currentInverseDynamicsJoint, Vector3d offset, String frameName)
   {
      ReferenceFrame parentFrame = currentInverseDynamicsJoint.getFrameAfterJoint();
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.setTranslationAndIdentityRotation(offset);
      ReferenceFrame beforeJointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);

      return beforeJointFrame;
   }

   private void compareWrenches(Wrench inputWrench, Wrench outputWrench, DenseMatrix64F selectionMatrix)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      inputWrench.getExpressedInFrame().checkReferenceFrameMatch(outputWrench.getExpressedInFrame());

      DenseMatrix64F inputWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      DenseMatrix64F outputWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      DenseMatrix64F selectedValues = new DenseMatrix64F(Wrench.SIZE, 1);

      inputWrench.getMatrix(inputWrenchMatrix);
      outputWrench.getMatrix(outputWrenchMatrix);

      double epsilon = 1e-4;
      int taskSize = selectionMatrix.getNumRows();
      int colIndex = 0;
      for (int i = 0; i < taskSize; i++)
         for (int j = colIndex; j < Wrench.SIZE; j++)
            if (selectionMatrix.get(i, j) == 1)
               selectedValues.set(j, 0, 1);

      // only compare the selected values
      for (int i = 0; i < Wrench.SIZE; i++)
      {
         if (selectedValues.get(i, 0) == 1)
            Assert.assertEquals(inputWrenchMatrix.get(i, 0), outputWrenchMatrix.get(i, 0), epsilon);
         else
            Assert.assertNotEquals(inputWrenchMatrix.get(i, 0), outputWrenchMatrix.get(i, 0), epsilon);
      }
   }

   private void compareWrenches(Wrench inputWrench, Wrench outputWrench)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      inputWrench.getExpressedInFrame().checkReferenceFrameMatch(outputWrench.getExpressedInFrame());

      double epsilon = 1e-4;
      JUnitTools.assertTuple3dEquals(inputWrench.getAngularPartCopy(), outputWrench.getAngularPartCopy(), epsilon);
      JUnitTools.assertTuple3dEquals(inputWrench.getLinearPartCopy(), outputWrench.getLinearPartCopy(), epsilon);
   }

   private class RobotLeg
   {
      private final RigidBody base;
      private final RigidBody endEffector;
      private final InverseDynamicsJoint rootJoint;

      public RobotLeg(RigidBody base, RigidBody endEffector, InverseDynamicsJoint rootJoint)
      {
         this.base = base;
         this.endEffector = endEffector;
         this.rootJoint = rootJoint;
      }

      public RigidBody getBase()
      {
         return base;
      }

      public RigidBody getEndEffector()
      {
         return endEffector;
      }

      public InverseDynamicsJoint getRootJoint()
      {
         return rootJoint;
      }
   }
}
