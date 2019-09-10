package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.OneDoFJointDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

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

   public static class UpperBodyWithTwoManipulators extends RobotDescription
   {
      private final Vector3D torsoYawOffset = new Vector3D(0.0, 0.0, 0.0);
      private final double torsoHeight = 0.5;
      private final double torsoWidth = 0.35;

      private final SideDependentList<Vector3D> shoulderYawOffset = new SideDependentList<>(side -> new Vector3D(0.0,
                                                                                                                 0.5 * side.negateIfRightSide(torsoWidth),
                                                                                                                 torsoHeight));
      private final SideDependentList<Vector3D> shoulderRollOffset = new SideDependentList<>(side -> new Vector3D(0.0, 0.0, 0.0));
      private final SideDependentList<Vector3D> shoulderPitchOffset = new SideDependentList<>(side -> new Vector3D(0.0, 0.0, 0.0));

      private final double upperArmLength = 0.35;
      private final double upperArmRadius = 0.025;

      private final SideDependentList<Vector3D> elbowPitchOffset = new SideDependentList<>(side -> new Vector3D(0.0, 0.0, upperArmLength));

      private final double lowerArmLength = 0.35;
      private final double lowerArmRadius = 0.025;

      private final SideDependentList<Vector3D> wristPitchOffset = new SideDependentList<>(side -> new Vector3D(0.0, 0.0, lowerArmLength));
      private final SideDependentList<Vector3D> wristRollOffset = new SideDependentList<>(side -> new Vector3D(0.0, 0.0, 0.0));
      private final SideDependentList<Vector3D> wristYawOffset = new SideDependentList<>(side -> new Vector3D(0.0, 0.0, 0.0));

      private final SideDependentList<AppearanceDefinition> armAppearances = new SideDependentList<>(YoAppearance.DarkRed(), YoAppearance.DarkMagenta());

      public UpperBodyWithTwoManipulators()
      {
         super("UpperBodyWithTwoManipulators");

         PinJointDescription torsoYaw = new PinJointDescription("torsoYaw", torsoYawOffset, Axis.Z);
         torsoYaw.setLimitStops(-Math.PI / 3.0, Math.PI / 3.0, 0.0, 0.0);

         LinkDescription torsoLink = new LinkDescription("torsoLink");
         torsoLink.setMass(5.0);
         torsoLink.setMomentOfInertia(createNullMOI());
         torsoLink.setLinkGraphics(createTorsoGraphics(torsoHeight, torsoWidth, 0.3 * torsoWidth, YoAppearance.AluminumMaterial()));

         addRootJoint(torsoYaw);
         torsoYaw.setLink(torsoLink);

         for (RobotSide robotSide : RobotSide.values)
         {
            String sidePrefix = robotSide.getCamelCaseName();
            PinJointDescription shoulderYaw = new PinJointDescription(sidePrefix + "ShoulderYaw", shoulderYawOffset.get(robotSide), Axis.Z);
            PinJointDescription shoulderRoll = new PinJointDescription(sidePrefix + "ShoulderRoll", shoulderRollOffset.get(robotSide), Axis.X);
            PinJointDescription shoulderPitch = new PinJointDescription(sidePrefix + "ShoulderPitch", shoulderPitchOffset.get(robotSide), Axis.Y);
            PinJointDescription elbowPitch = new PinJointDescription(sidePrefix + "ElbowPitch", elbowPitchOffset.get(robotSide), Axis.Y);
            PinJointDescription wristPitch = new PinJointDescription(sidePrefix + "WristPitch", wristPitchOffset.get(robotSide), Axis.Y);
            PinJointDescription wristRoll = new PinJointDescription(sidePrefix + "WristRoll", wristRollOffset.get(robotSide), Axis.X);
            PinJointDescription wristYaw = new PinJointDescription(sidePrefix + "WristYaw", wristYawOffset.get(robotSide), Axis.Z);

            shoulderYaw.setLimitStops(-Math.PI, Math.PI, 0.0, 0.0);
            shoulderRoll.setLimitStops(-Math.PI, Math.PI, 0.0, 0.0);
            shoulderPitch.setLimitStops(-Math.PI, Math.PI, 0.0, 0.0);
            elbowPitch.setLimitStops(0.0, 2.0 / 3.0 * Math.PI, 0.0, 0.0);
            wristPitch.setLimitStops(-Math.PI, Math.PI, 0.0, 0.0);
            wristRoll.setLimitStops(-Math.PI, Math.PI, 0.0, 0.0);
            wristYaw.setLimitStops(-Math.PI, Math.PI, 0.0, 0.0);

            LinkDescription shoulderYawLink = new LinkDescription(sidePrefix + "ShoulderYawLink");
            shoulderYawLink.setMass(0.1);
            shoulderYawLink.setMomentOfInertia(createNullMOI());

            LinkDescription shoulderRollLink = new LinkDescription(sidePrefix + "ShoulderRollLink");
            shoulderRollLink.setMass(0.1);
            shoulderRollLink.setMomentOfInertia(createNullMOI());

            LinkDescription upperArmLink = new LinkDescription(sidePrefix + "UpperArmLink");
            upperArmLink.setMass(1.0);
            upperArmLink.setMomentOfInertia(createNullMOI());
            upperArmLink.setLinkGraphics(createArmGraphic(upperArmLength, upperArmRadius, armAppearances.get(robotSide)));

            LinkDescription lowerArmLink = new LinkDescription(sidePrefix + "LowerArmLink");
            lowerArmLink.setMass(1.0);
            lowerArmLink.setMomentOfInertia(createNullMOI());
            lowerArmLink.setLinkGraphics(createArmGraphic(lowerArmLength,
                                                          lowerArmRadius,
                                                          new YoAppearanceRGBColor(armAppearances.get(robotSide).getAwtColor().brighter().brighter().brighter(),
                                                                                   0.0)));

            LinkDescription wristPitchLink = new LinkDescription(sidePrefix + "WristPitchLink");
            wristPitchLink.setMass(0.1);
            wristPitchLink.setMomentOfInertia(createNullMOI());

            LinkDescription wristRollLink = new LinkDescription(sidePrefix + "WristRollLink");
            wristRollLink.setMass(0.1);
            wristRollLink.setMomentOfInertia(createNullMOI());

            LinkDescription handLink = new LinkDescription(sidePrefix + "HandLink");
            handLink.setMass(1.0);
            handLink.setMomentOfInertia(createNullMOI());
            handLink.setLinkGraphics(createHandGraphics());

            torsoYaw.addJoint(shoulderYaw);
            shoulderYaw.setLink(shoulderYawLink);
            shoulderYaw.addJoint(shoulderRoll);
            shoulderRoll.setLink(shoulderRollLink);
            shoulderRoll.addJoint(shoulderPitch);
            shoulderPitch.setLink(upperArmLink);
            shoulderPitch.addJoint(elbowPitch);
            elbowPitch.setLink(lowerArmLink);
            elbowPitch.addJoint(wristPitch);
            wristPitch.setLink(wristPitchLink);
            wristPitch.addJoint(wristRoll);
            wristRoll.setLink(wristRollLink);
            wristRoll.addJoint(wristYaw);
            wristYaw.setLink(handLink);
         }
      }
   }

   public static Pair<FloatingJointBasics, OneDoFJointBasics[]> createInverseDynamicsRobot(RobotDescription robotDescription)
   {
      RigidBodyBasics predecessor;

      RigidBodyBasics rootBody = new RigidBody("rootBody", ReferenceFrame.getWorldFrame());
      FloatingJointBasics rootJoint;

      if (robotDescription.getRootJoints().get(0) instanceof FloatingJointBasics)
      {
         FloatingJointDescription rootJointDescription = (FloatingJointDescription) robotDescription.getRootJoints().get(0);
         rootJoint = new SixDoFJoint(rootJointDescription.getName(), rootBody);

         LinkDescription linkDescription = rootJointDescription.getLink();
         predecessor = new RigidBody(rootJointDescription.getName(),
                                     rootJoint,
                                     linkDescription.getMomentOfInertiaCopy(),
                                     linkDescription.getMass(),
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

      return new ImmutablePair<>(rootJoint, MultiBodySystemTools.filterJoints(MultiBodySystemTools.collectSubtreeJoints(predecessor), OneDoFJointBasics.class));
   }

   protected static void addJointsRecursively(OneDoFJointDescription joint, RigidBodyBasics parentBody)
   {
      Vector3D jointAxis = new Vector3D();
      joint.getJointAxis(jointAxis);

      Vector3D offset = new Vector3D();
      joint.getOffsetFromParentJoint(offset);

      OneDoFJointBasics inverseDynamicsJoint;

      if (joint instanceof PinJointDescription)
      {
         inverseDynamicsJoint = new RevoluteJoint(joint.getName(), parentBody, offset, jointAxis);
      }
      else if (joint instanceof SliderJointDescription)
      {
         inverseDynamicsJoint = new PrismaticJoint(joint.getName(), parentBody, offset, jointAxis);
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

      RigidBodyBasics rigidBody = new RigidBody(childLink.getName(), inverseDynamicsJoint, inertia, mass, comOffset);

      for (JointDescription sdfJoint : joint.getChildrenJoints())
      {
         addJointsRecursively((OneDoFJointDescription) sdfJoint, rigidBody);
      }
   }

   private static LinkGraphicsDescription createTorsoGraphics(double torsoHeight, double torsoWidth, double torsoThickness, AppearanceDefinition appearance)
   {
      LinkGraphicsDescription graphics = new LinkGraphicsDescription();
      graphics.addGenTruncatedCone(torsoHeight, torsoThickness, 0.7 * torsoWidth, 0.7 * torsoThickness, 0.5 * torsoWidth, appearance);
      graphics.translate(0.0, 0.0, 0.5 * torsoHeight);
      return graphics;
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
