package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.OneDoFJointDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;

public class KinematicsToolboxControllerTestRobots
{
   public static class SevenDoFArm extends RobotDescription
   {
      private final Vector3D shoulderYawOffset = new Vector3D(0.0, 0.0, 0.3);
      private final Vector3D shoulderRollOffset = new Vector3D(0.0, 0.0, 0.0);
      private final Vector3D shoulderPitchOffset = new Vector3D(0.0, 0.0, 0.0);

      private final double upperArmLength = 0.35;
      private final double upperArmRadius = 0.025;

      private final Vector3D elbowPitchOffset = new Vector3D(0.0, 0.0, upperArmLength);

      private final double lowerArmLength = 0.35;
      private final double lowerArmRadius = 0.025;

      private final Vector3D wristPitchOffset = new Vector3D(0.0, 0.0, lowerArmLength);
      private final Vector3D wristRollOffset = new Vector3D(0.0, 0.0, 0.0);
      private final Vector3D wristYawOffset = new Vector3D(0.0, 0.0, 0.0);

      public SevenDoFArm()
      {
         super("7DoFArm");

         PinJointDescription scsShoulderYaw = new PinJointDescription("shoulderYaw", shoulderYawOffset, Axis.Z);
         PinJointDescription scsShoulderRoll = new PinJointDescription("shoulderRoll", shoulderRollOffset, Axis.X);
         PinJointDescription scsShoulderPitch = new PinJointDescription("shoulderPitch", shoulderPitchOffset, Axis.Y);
         PinJointDescription scsElbowPitch = new PinJointDescription("elbowPitch", elbowPitchOffset, Axis.Y);
         PinJointDescription scsWristPitch = new PinJointDescription("wristPitch", wristPitchOffset, Axis.Y);
         PinJointDescription scsWristRoll = new PinJointDescription("wristRoll", wristRollOffset, Axis.X);
         PinJointDescription scsWristYaw = new PinJointDescription("wristYaw", wristYawOffset, Axis.Z);

         scsShoulderYaw.setLimitStops(-Math.PI, Math.PI, 0.0, 0.0);
         scsShoulderRoll.setLimitStops(-Math.PI, Math.PI, 0.0, 0.0);
         scsShoulderPitch.setLimitStops(-Math.PI, Math.PI, 0.0, 0.0);
         scsElbowPitch.setLimitStops(0.0, 2.0 / 3.0 * Math.PI, 0.0, 0.0);
         scsWristPitch.setLimitStops(-Math.PI, Math.PI, 0.0, 0.0);
         scsWristRoll.setLimitStops(-Math.PI, Math.PI, 0.0, 0.0);
         scsWristYaw.setLimitStops(-Math.PI, Math.PI, 0.0, 0.0);

         LinkDescription scsShoulderYawLink = new LinkDescription("shoulderYawLink");
         scsShoulderYawLink.setMass(0.1);
         scsShoulderYawLink.setMomentOfInertia(createNullMOI());

         LinkDescription scsShoulderRollLink = new LinkDescription("shoulderRollLink");
         scsShoulderRollLink.setMass(0.1);
         scsShoulderRollLink.setMomentOfInertia(createNullMOI());

         LinkDescription scsUpperArmLink = new LinkDescription("upperArmLink");
         scsUpperArmLink.setMass(1.0);
         scsUpperArmLink.setMomentOfInertia(createNullMOI());
         scsUpperArmLink.setLinkGraphics(createArmGraphic(upperArmLength, upperArmRadius, YoAppearance.Red()));

         LinkDescription scsLowerArmLink = new LinkDescription("lowerArmLink");
         scsLowerArmLink.setMass(1.0);
         scsLowerArmLink.setMomentOfInertia(createNullMOI());
         scsLowerArmLink.setLinkGraphics(createArmGraphic(lowerArmLength, lowerArmRadius, YoAppearance.Green()));

         LinkDescription scsWristPitchLink = new LinkDescription("wristPitchLink");
         scsWristPitchLink.setMass(0.1);
         scsWristPitchLink.setMomentOfInertia(createNullMOI());

         LinkDescription scsWristRollLink = new LinkDescription("wristRollLink");
         scsWristRollLink.setMass(0.1);
         scsWristRollLink.setMomentOfInertia(createNullMOI());

         LinkDescription scsHandLink = new LinkDescription("handLink");
         scsHandLink.setMass(1.0);
         scsHandLink.setMomentOfInertia(createNullMOI());
         scsHandLink.setLinkGraphics(createHandGraphics());

         addRootJoint(scsShoulderYaw);
         scsShoulderYaw.setLink(scsShoulderYawLink);
         scsShoulderYaw.addJoint(scsShoulderRoll);
         scsShoulderRoll.setLink(scsShoulderRollLink);
         scsShoulderRoll.addJoint(scsShoulderPitch);
         scsShoulderPitch.setLink(scsUpperArmLink);
         scsShoulderPitch.addJoint(scsElbowPitch);
         scsElbowPitch.setLink(scsLowerArmLink);
         scsElbowPitch.addJoint(scsWristPitch);
         scsWristPitch.setLink(scsWristPitchLink);
         scsWristPitch.addJoint(scsWristRoll);
         scsWristRoll.setLink(scsWristRollLink);
         scsWristRoll.addJoint(scsWristYaw);
         scsWristYaw.setLink(scsHandLink);
      }
   }

   public static Pair<FloatingInverseDynamicsJoint, OneDoFJoint[]> createInverseDynamicsRobot(RobotDescription robotDescription)
   {
      RigidBody predecessor;

      RigidBody rootBody = new RigidBody("rootBody", ReferenceFrame.getWorldFrame());
      FloatingInverseDynamicsJoint rootJoint;

      if (robotDescription.getRootJoints().get(0) instanceof FloatingInverseDynamicsJoint)
      {
         FloatingJointDescription rootJointDescription = (FloatingJointDescription) robotDescription.getRootJoints().get(0);
         rootJoint = new SixDoFJoint(rootJointDescription.getName(), rootBody);

         LinkDescription linkDescription = rootJointDescription.getLink();
         predecessor = ScrewTools.addRigidBody(rootJointDescription.getName(), rootJoint, linkDescription.getMomentOfInertiaCopy(), linkDescription.getMass(),
                                               linkDescription.getCenterOfMassOffset());

         for (JointDescription jointDescription : rootJointDescription.getChildrenJoints())
         {
            addJointsRecursively((OneDoFJointDescription) jointDescription, predecessor);
         }
      }
      else
      {
         rootJoint = null;
         predecessor = rootBody;
         addJointsRecursively((OneDoFJointDescription) robotDescription.getRootJoints().get(0), predecessor);
      }

      return new ImmutablePair<>(rootJoint, ScrewTools.filterJoints(ScrewTools.computeSubtreeJoints(predecessor), OneDoFJoint.class));
   }

   

   protected static void addJointsRecursively(OneDoFJointDescription joint, RigidBody parentBody)
   {
      Vector3D jointAxis = new Vector3D();
      joint.getJointAxis(jointAxis);

      Vector3D offset = new Vector3D();
      joint.getOffsetFromParentJoint(offset);

      OneDoFJoint inverseDynamicsJoint;

      if (joint instanceof PinJointDescription)
      {
         inverseDynamicsJoint = ScrewTools.addRevoluteJoint(joint.getName(), parentBody, offset, jointAxis);
      }
      else if (joint instanceof SliderJointDescription)
      {
         inverseDynamicsJoint = ScrewTools.addPrismaticJoint(joint.getName(), parentBody, offset, jointAxis);
      }
      else
      {
         throw new RuntimeException("Must be either Pin or Slider here!");
      }

      if (joint.containsLimitStops())
      {
         double[] limitStopParameters = joint.getLimitStopParameters();
         inverseDynamicsJoint.setJointLimitLower(limitStopParameters[0]);
         inverseDynamicsJoint.setJointLimitUpper(limitStopParameters[1]);
      }

      LinkDescription childLink = joint.getLink();

      double mass = childLink.getMass();
      Vector3D comOffset = new Vector3D(childLink.getCenterOfMassOffset());
      Matrix3D inertia = childLink.getMomentOfInertiaCopy();

      RigidBody rigidBody = ScrewTools.addRigidBody(childLink.getName(), inverseDynamicsJoint, inertia, mass, comOffset);

      for (JointDescription sdfJoint : joint.getChildrenJoints())
      {
         addJointsRecursively((OneDoFJointDescription) sdfJoint, rigidBody);
      }
   }

   private static LinkGraphicsDescription createHandGraphics()
   {
      LinkGraphicsDescription graphics = new LinkGraphicsDescription();
      graphics.addSphere(0.025, YoAppearance.Grey());
      graphics.translate(0.0, 0.0, 0.05);
      graphics.addEllipsoid(0.04, 0.01, 0.1);
      return graphics;
   }

   private static LinkGraphicsDescription createArmGraphic(double length, double radius, AppearanceDefinition appearance)
   {
      LinkGraphicsDescription graphics = new LinkGraphicsDescription();
      graphics.addSphere(1.2 * radius, YoAppearance.Grey());
      graphics.addCylinder(length, radius, appearance);
      return graphics;
   }

   private static Matrix3D createNullMOI()
   {
      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setIdentity();
      momentOfInertia.scale(1.0e-4);
      return momentOfInertia;
   }
}
