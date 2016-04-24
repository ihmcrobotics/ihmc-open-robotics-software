package us.ihmc.commonWalkingControlModules.controllerCore;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
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

   public static final double BASE_HEIGHT = 1.0;
   public static final double BASE_RAD = 0.1;

   public static final double SHOULDER_DIFFERENTIAL_HEIGHT = 0.05;
   public static final double SHOULDER_DIFFERENTIAL_WIDTH = 0.075;

   public static final double UPPER_ARM_LENGTH = 23.29 * INCHES;
   public static final double UPPER_ARM_RAD = 0.05;
   public static final double UPPER_ARM_MASS = 6.7 * POUNDS;

   public static final double FOREARM_LENGTH = 9.1 * INCHES;
   public static final double FOREARM_RAD = 0.03;
   public static final double FOREARM_MASS = 1.0 * POUNDS;

   public static final double WRIST_DIFFERENTIAL_HEIGHT = 0.025;
   public static final double WRIST_DIFFERENTIAL_WIDTH = 0.0375;

   public static final double GRIPPER_LENGTH = 0.08;
   public static final double GRIPPER_COM_OFFSET = 3.0 * INCHES;
   public static final double GRIPPER_RAD = 0.05;
   public static final double GRIPPER_MASS = 3.0 * POUNDS;

   private final Random random = new Random(100L);

   @DeployableTestMethod
   @Test(timeout = 30000)
   public void testPlanarTorqueComputation()
   {
      double gravity = -9.81;

      RobotArm robotArm = createPlanarRobotArm(gravity);
      RigidBody base = robotArm.getBase();
      RigidBody endEffector = robotArm.getEndEffector();

      GeometricJacobianHolder geometricJacobianHolder = new GeometricJacobianHolder();
      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), base);

      RigidBody[] endEffectors = {endEffector};
      InverseDynamicsJoint[] controlledJoints = ScrewTools.createJointPath(base, endEffector);

      VirtualModelController virtualModelController = new VirtualModelController(geometricJacobianHolder, base);
      virtualModelController.registerEndEffector(base, endEffector);

      Wrench desiredWrench = new Wrench(endEffector.getBodyFixedFrame(), endEffector.getBodyFixedFrame());
      desiredWrench.setLinearPartX(5.0);
      desiredWrench.setLinearPartY(0.0);
      desiredWrench.setLinearPartZ(10);
      desiredWrench.setAngularPartX(0.0);
      desiredWrench.setAngularPartY(2.0);
      desiredWrench.setAngularPartZ(0.0);

      virtualModelController.submitEndEffectorVirtualWrench(endEffector, desiredWrench);

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
      long jacobianID = geometricJacobianHolder.getOrCreateGeometricJacobian(controlledJoints, endEffector.getBodyFixedFrame());
      DenseMatrix64F jacobianMatrix = geometricJacobianHolder.getJacobian(jacobianID).getJacobianMatrix();
      DenseMatrix64F appliedWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      CommonOps.mult(jacobianMatrix, jointEffortMatrix, appliedWrenchMatrix);
      Wrench appliedWrench = new Wrench(endEffector.getBodyFixedFrame(), endEffector.getBodyFixedFrame(), appliedWrenchMatrix);

      compareWrenches(desiredWrench, appliedWrench);
   }

   private RobotArm createPlanarRobotArm(double gravity)
   {
      Robot robotArm = new Robot("robotArm");
      robotArm.setGravity(gravity);

      Link baseLink = base();
      robotArm.addStaticLink(baseLink);
      ReferenceFrame baseFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(), new RigidBodyTransform());
      RigidBody baseBody = new RigidBody("base", baseFrame);

      Vector3d shoulderPitchOffset = new Vector3d(0.0, 0.0, SHOULDER_DIFFERENTIAL_HEIGHT);
      PinJoint shoulder_pitch = new PinJoint("shoulder_pitch", shoulderPitchOffset, robotArm, Axis.Y);
      shoulder_pitch.setQ(random.nextDouble());
      Link upper_arm = upper_arm();
      shoulder_pitch.setLink(upper_arm);
      robotArm.addRootJoint(shoulder_pitch);
      RevoluteJoint shoulderPitch = ScrewTools.addRevoluteJoint("shoulder_pitch", baseBody, shoulderPitchOffset, Y);
      RigidBody upperArmBody = copyLinkAsRigidBody(upper_arm, shoulderPitch, "upper_arm");

      Vector3d elbowPitchOffset = new Vector3d(0.0, 0.0, UPPER_ARM_LENGTH);
      PinJoint elbow_pitch = new PinJoint("elbow_pitch", elbowPitchOffset, robotArm, Axis.Y);
      elbow_pitch.setQ(random.nextDouble());
      Link forearm = forearm();
      elbow_pitch.setLink(forearm);
      shoulder_pitch.addJoint(elbow_pitch);
      RevoluteJoint elbowPitch = ScrewTools.addRevoluteJoint("elbow_pitch", upperArmBody, elbowPitchOffset, Y);
      RigidBody forearmBody = copyLinkAsRigidBody(forearm, elbowPitch, "forearm");

      Vector3d wristPitchOffset = new Vector3d(0.0, 0.0, FOREARM_LENGTH);
      PinJoint wrist_pitch = new PinJoint("wrist_pitch", wristPitchOffset, robotArm, Axis.Y);
      wrist_pitch.setQ(random.nextDouble());
      Link gripper = gripper();
      wrist_pitch.setLink(gripper);
      elbow_pitch.addJoint(wrist_pitch);
      RevoluteJoint wristPitch = ScrewTools.addRevoluteJoint("wrist_pitch", forearmBody, wristPitchOffset, Z);
      RigidBody gripperBody = copyLinkAsRigidBody(gripper, wristPitch, "gripper");

      return new RobotArm(baseBody, gripperBody);
   }

   private RobotArm createRobotArm(double gravity)
   {
      Robot robotArm = new Robot("robotArm");
      robotArm.setGravity(gravity);

      Link baseLink = base();
      robotArm.addStaticLink(baseLink);
      ReferenceFrame baseFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(), new RigidBodyTransform());
      RigidBody baseBody = new RigidBody("base", baseFrame);

      Vector3d shoulderYawOffset = new Vector3d(0.0, 0.0, BASE_HEIGHT);
      PinJoint shoulder_yaw = new PinJoint("shoulder_yaw", shoulderYawOffset, robotArm, Axis.X);
      shoulder_yaw.setQ(random.nextDouble());
      Link shoulder_differential = shoulder_differential();
      shoulder_yaw.setLink(shoulder_differential);
      robotArm.addRootJoint(shoulder_yaw);
      RevoluteJoint shoulderYaw = ScrewTools.addRevoluteJoint("shoulder_yaw", baseBody, shoulderYawOffset, X);
      RigidBody shoulderDifferentialBody = copyLinkAsRigidBody(shoulder_differential, shoulderYaw, "shoulder_differential");

      Vector3d shoulderPitchOffset = new Vector3d(0.0, 0.0, SHOULDER_DIFFERENTIAL_HEIGHT);
      PinJoint shoulder_pitch = new PinJoint("shoulder_pitch", shoulderPitchOffset, robotArm, Axis.Y);
      shoulder_pitch.setQ(random.nextDouble());
      Link upper_arm = upper_arm();
      shoulder_pitch.setLink(upper_arm);
      shoulder_yaw.addJoint(shoulder_pitch);
      RevoluteJoint shoulderPitch = ScrewTools.addRevoluteJoint("shoulder_pitch", shoulderDifferentialBody, shoulderPitchOffset, Y);
      RigidBody upperArmBody = copyLinkAsRigidBody(upper_arm, shoulderPitch, "upper_arm");

      Vector3d elbowPitchOffset = new Vector3d(0.0, 0.0, UPPER_ARM_LENGTH);
      PinJoint elbow_pitch = new PinJoint("elbow_pitch", elbowPitchOffset, robotArm, Axis.Y);
      elbow_pitch.setQ(random.nextDouble());
      Link forearm = forearm();
      elbow_pitch.setLink(forearm);
      shoulder_pitch.addJoint(elbow_pitch);
      RevoluteJoint elbowPitch = ScrewTools.addRevoluteJoint("elbow_pitch", upperArmBody, elbowPitchOffset, Y);
      RigidBody forearmBody = copyLinkAsRigidBody(forearm, elbowPitch, "forearm");

      Vector3d wristPitchOffset = new Vector3d(0.0, 0.0, FOREARM_LENGTH);
      PinJoint wrist_pitch = new PinJoint("wrist_pitch", wristPitchOffset, robotArm, Axis.Y);
      wrist_pitch.setQ(random.nextDouble());
      Link wrist_differential = wrist_differential();
      wrist_pitch.setLink(wrist_differential);
      elbow_pitch.addJoint(wrist_pitch);
      RevoluteJoint wristPitch = ScrewTools.addRevoluteJoint("wrist_pitch", forearmBody, wristPitchOffset, Y);
      RigidBody wristDifferentialBody = copyLinkAsRigidBody(wrist_differential, wristPitch, "wrist_differential");

      Vector3d wristYawOffset = new Vector3d();
      PinJoint wrist_yaw = new PinJoint("wrist_yaw", wristYawOffset, robotArm, Axis.X);
      wrist_yaw.setQ(random.nextDouble());
      Link wrist_differential2 = wrist_differential();
      wrist_yaw.setLink(wrist_differential2);
      wrist_pitch.addJoint(wrist_yaw);
      RevoluteJoint wristYaw = ScrewTools.addRevoluteJoint("wrist_yaw", wristDifferentialBody, wristYawOffset, X);
      RigidBody wristDifferential2Body = copyLinkAsRigidBody(wrist_differential2, wristYaw, "wrist_differential");

      Vector3d wristRollOffset = new Vector3d(0.0, 0.0, WRIST_DIFFERENTIAL_HEIGHT);
      PinJoint wrist_roll = new PinJoint("wrist_roll", wristRollOffset, robotArm, Axis.Z);
      wrist_roll.setQ(random.nextDouble());
      Link gripper = gripper();
      wrist_roll.setLink(gripper);
      wrist_yaw.addJoint(wrist_roll);
      RevoluteJoint wristRoll = ScrewTools.addRevoluteJoint("wrist_roll", wristDifferential2Body, wristRollOffset, Z);
      RigidBody gripperBody = copyLinkAsRigidBody(gripper, wristRoll, "gripper");

      return new RobotArm(baseBody, gripperBody);
   }

   private Link base()
   {
      AppearanceDefinition baseAppearance = YoAppearance.Blue();

      Link ret = new Link("base");

      ret.setMass(100.0);
      ret.setMomentOfInertia(1.0, 1.0, 1.0);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCoordinateSystem(1.0);
      linkGraphics.addCylinder(BASE_HEIGHT, BASE_RAD, baseAppearance);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link shoulder_differential()
   {
      Link ret = new Link("shoulder_differential");

      ret.setMass(0.1);
      ret.setMomentOfInertia(0.005, 0.005, 0.005);
      ret.setComOffset(0.0, 0.0, SHOULDER_DIFFERENTIAL_HEIGHT / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCube(SHOULDER_DIFFERENTIAL_WIDTH, SHOULDER_DIFFERENTIAL_WIDTH, SHOULDER_DIFFERENTIAL_HEIGHT);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   private Link upper_arm()
   {
      AppearanceDefinition upperArmApp = YoAppearance.Green();

      Link ret = new Link("upper_arm");

      ret.setMass(UPPER_ARM_MASS);    // 2.35);
      ret.setMomentOfInertia(0.0437, 0.0437, 0.0054);
      ret.setComOffset(0.0, 0.0, UPPER_ARM_LENGTH / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(UPPER_ARM_LENGTH, UPPER_ARM_RAD, upperArmApp);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   private Link forearm()
   {
      AppearanceDefinition forearmApp = YoAppearance.Red();

      Link ret = new Link("forearm");

      ret.setMass(FOREARM_MASS);    // 0.864);
      ret.setMomentOfInertia(0.00429, 0.00429, 0.00106);
      ret.setComOffset(0.0, 0.0, FOREARM_LENGTH / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(FOREARM_LENGTH, FOREARM_RAD, forearmApp);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link wrist_differential()
   {
      Link ret = new Link("wrist_differential");

      ret.setMass(0.1);
      ret.setMomentOfInertia(0.005, 0.005, 0.005);
      ret.setComOffset(0.0, 0.0, WRIST_DIFFERENTIAL_HEIGHT / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCube(WRIST_DIFFERENTIAL_WIDTH, WRIST_DIFFERENTIAL_WIDTH, WRIST_DIFFERENTIAL_HEIGHT);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link gripper()
   {
      AppearanceDefinition gripperApp = YoAppearance.PlaneMaterial();

      Link ret = new Link("gripper");

      ret.setMass(GRIPPER_MASS);    // 0.207);
      ret.setMomentOfInertia(0.00041, 0.00041, 0.00001689);
      ret.setComOffset(0.0, 0.0, GRIPPER_COM_OFFSET);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(GRIPPER_LENGTH, GRIPPER_RAD, gripperApp);

      linkGraphics.translate(0.05, 0.0, GRIPPER_LENGTH);
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

   private void compareWrenches(Wrench inputWrench, Wrench outputWrench)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      inputWrench.getExpressedInFrame().checkReferenceFrameMatch(outputWrench.getExpressedInFrame());

      double epsilon = 1e-12; //3;
      JUnitTools.assertTuple3dEquals(inputWrench.getAngularPartCopy(), outputWrench.getAngularPartCopy(), epsilon);
      JUnitTools.assertTuple3dEquals(inputWrench.getLinearPartCopy(), outputWrench.getLinearPartCopy(), epsilon);
   }

   private class RobotArm
   {
      private final RigidBody base;
      private final RigidBody endEffector;

      public RobotArm(RigidBody base, RigidBody endEffector)
      {
         this.base = base;
         this.endEffector = endEffector;
      }

      public RigidBody getBase()
      {
         return base;
      }

      public RigidBody getEndEffector()
      {
         return endEffector;
      }
   }
}
