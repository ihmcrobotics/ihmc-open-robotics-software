package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;

public class KinematicsToolboxControllerTestRobots
{
   public static class SevenDoFArm extends RobotDefinition
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

         RevoluteJointDefinition scsShoulderYaw = new RevoluteJointDefinition("shoulderYaw", shoulderYawOffset, Axis3D.Z);
         RevoluteJointDefinition scsShoulderRoll = new RevoluteJointDefinition("shoulderRoll", shoulderRollOffset, Axis3D.X);
         RevoluteJointDefinition scsShoulderPitch = new RevoluteJointDefinition("shoulderPitch", shoulderPitchOffset, Axis3D.Y);
         RevoluteJointDefinition scsElbowPitch = new RevoluteJointDefinition("elbowPitch", elbowPitchOffset, Axis3D.Y);
         RevoluteJointDefinition scsWristPitch = new RevoluteJointDefinition("wristPitch", wristPitchOffset, Axis3D.Y);
         RevoluteJointDefinition scsWristRoll = new RevoluteJointDefinition("wristRoll", wristRollOffset, Axis3D.X);
         RevoluteJointDefinition scsWristYaw = new RevoluteJointDefinition("wristYaw", wristYawOffset, Axis3D.Z);

         scsShoulderYaw.setPositionLimits(-Math.PI, Math.PI);
         scsShoulderRoll.setPositionLimits(-Math.PI, Math.PI);
         scsShoulderPitch.setPositionLimits(-Math.PI, Math.PI);
         scsElbowPitch.setPositionLimits(0.0, 2.0 / 3.0 * Math.PI);
         scsWristPitch.setPositionLimits(-Math.PI, Math.PI);
         scsWristRoll.setPositionLimits(-Math.PI, Math.PI);
         scsWristYaw.setPositionLimits(-Math.PI, Math.PI);

         RigidBodyDefinition rootBody = new RigidBodyDefinition("root");

         RigidBodyDefinition scsShoulderYawLink = new RigidBodyDefinition("shoulderYawLink");
         scsShoulderYawLink.setMass(0.1);
         scsShoulderYawLink.setMomentOfInertia(createNullMOI());

         RigidBodyDefinition scsShoulderRollLink = new RigidBodyDefinition("shoulderRollLink");
         scsShoulderRollLink.setMass(0.1);
         scsShoulderRollLink.setMomentOfInertia(createNullMOI());

         RigidBodyDefinition scsUpperArmLink = new RigidBodyDefinition("upperArmLink");
         scsUpperArmLink.setMass(1.0);
         scsUpperArmLink.setMomentOfInertia(createNullMOI());
         scsUpperArmLink.setVisualDefinitions(createArmVisuals(upperArmLength, upperArmRadius, ColorDefinitions.Red()));

         RigidBodyDefinition scsLowerArmLink = new RigidBodyDefinition("lowerArmLink");
         scsLowerArmLink.setMass(1.0);
         scsLowerArmLink.setMomentOfInertia(createNullMOI());
         scsLowerArmLink.setVisualDefinitions(createArmVisuals(lowerArmLength, lowerArmRadius, ColorDefinitions.Green()));

         RigidBodyDefinition scsWristPitchLink = new RigidBodyDefinition("wristPitchLink");
         scsWristPitchLink.setMass(0.1);
         scsWristPitchLink.setMomentOfInertia(createNullMOI());

         RigidBodyDefinition scsWristRollLink = new RigidBodyDefinition("wristRollLink");
         scsWristRollLink.setMass(0.1);
         scsWristRollLink.setMomentOfInertia(createNullMOI());

         RigidBodyDefinition scsHandLink = new RigidBodyDefinition("handLink");
         scsHandLink.setMass(1.0);
         scsHandLink.setMomentOfInertia(createNullMOI());
         scsHandLink.setVisualDefinitions(createHandVisuals());

         setRootBodyDefinition(rootBody);
         rootBody.addChildJoint(scsShoulderYaw);
         scsShoulderYaw.setSuccessor(scsShoulderYawLink);
         scsShoulderYawLink.addChildJoint(scsShoulderRoll);
         scsShoulderRoll.setSuccessor(scsShoulderRollLink);
         scsShoulderRollLink.addChildJoint(scsShoulderPitch);
         scsShoulderPitch.setSuccessor(scsUpperArmLink);
         scsUpperArmLink.addChildJoint(scsElbowPitch);
         scsElbowPitch.setSuccessor(scsLowerArmLink);
         scsLowerArmLink.addChildJoint(scsWristPitch);
         scsWristPitch.setSuccessor(scsWristPitchLink);
         scsWristPitchLink.addChildJoint(scsWristRoll);
         scsWristRoll.setSuccessor(scsWristRollLink);
         scsWristRollLink.addChildJoint(scsWristYaw);
         scsWristYaw.setSuccessor(scsHandLink);
      }
   }

   public static class UpperBodyWithTwoManipulators extends RobotDefinition
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

      private final SideDependentList<ColorDefinition> armColors = new SideDependentList<>(ColorDefinitions.DarkRed(), ColorDefinitions.DarkMagenta());

      public UpperBodyWithTwoManipulators()
      {
         super("UpperBodyWithTwoManipulators");

         RigidBodyDefinition rootBody = new RigidBodyDefinition("root");
         setRootBodyDefinition(rootBody);
         RevoluteJointDefinition torsoYaw = new RevoluteJointDefinition("torsoYaw", torsoYawOffset, Axis3D.Z);
         torsoYaw.setPositionLimits(-Math.PI / 3.0, Math.PI / 3.0);

         RigidBodyDefinition torsoLink = new RigidBodyDefinition("torsoLink");
         torsoLink.setMass(5.0);
         torsoLink.setMomentOfInertia(createNullMOI());
         torsoLink.setVisualDefinitions(createTorsoVisuals(torsoHeight, torsoWidth, 0.3 * torsoWidth, ColorDefinitions.Grey()));

         rootBody.addChildJoint(torsoYaw);
         torsoYaw.setSuccessor(torsoLink);

         for (RobotSide robotSide : RobotSide.values)
         {
            String sidePrefix = robotSide.getCamelCaseName();
            RevoluteJointDefinition shoulderYaw = new RevoluteJointDefinition(sidePrefix + "ShoulderYaw", shoulderYawOffset.get(robotSide), Axis3D.Z);
            RevoluteJointDefinition shoulderRoll = new RevoluteJointDefinition(sidePrefix + "ShoulderRoll", shoulderRollOffset.get(robotSide), Axis3D.X);
            RevoluteJointDefinition shoulderPitch = new RevoluteJointDefinition(sidePrefix + "ShoulderPitch", shoulderPitchOffset.get(robotSide), Axis3D.Y);
            RevoluteJointDefinition elbowPitch = new RevoluteJointDefinition(sidePrefix + "ElbowPitch", elbowPitchOffset.get(robotSide), Axis3D.Y);
            RevoluteJointDefinition wristPitch = new RevoluteJointDefinition(sidePrefix + "WristPitch", wristPitchOffset.get(robotSide), Axis3D.Y);
            RevoluteJointDefinition wristRoll = new RevoluteJointDefinition(sidePrefix + "WristRoll", wristRollOffset.get(robotSide), Axis3D.X);
            RevoluteJointDefinition wristYaw = new RevoluteJointDefinition(sidePrefix + "WristYaw", wristYawOffset.get(robotSide), Axis3D.Z);

            shoulderYaw.setPositionLimits(-Math.PI, Math.PI);
            shoulderRoll.setPositionLimits(-Math.PI, Math.PI);
            shoulderPitch.setPositionLimits(-Math.PI, Math.PI);
            elbowPitch.setPositionLimits(0.0, 2.0 / 3.0 * Math.PI);
            wristPitch.setPositionLimits(-Math.PI, Math.PI);
            wristRoll.setPositionLimits(-Math.PI, Math.PI);
            wristYaw.setPositionLimits(-Math.PI, Math.PI);

            RigidBodyDefinition shoulderYawLink = new RigidBodyDefinition(sidePrefix + "ShoulderYawLink");
            shoulderYawLink.setMass(0.1);
            shoulderYawLink.setMomentOfInertia(createNullMOI());

            RigidBodyDefinition shoulderRollLink = new RigidBodyDefinition(sidePrefix + "ShoulderRollLink");
            shoulderRollLink.setMass(0.1);
            shoulderRollLink.setMomentOfInertia(createNullMOI());

            RigidBodyDefinition upperArmLink = new RigidBodyDefinition(sidePrefix + "UpperArmLink");
            upperArmLink.setMass(1.0);
            upperArmLink.setMomentOfInertia(createNullMOI());
            upperArmLink.setVisualDefinitions(createArmVisuals(upperArmLength, upperArmRadius, armColors.get(robotSide)));

            RigidBodyDefinition lowerArmLink = new RigidBodyDefinition(sidePrefix + "LowerArmLink");
            lowerArmLink.setMass(1.0);
            lowerArmLink.setMomentOfInertia(createNullMOI());
            lowerArmLink.setVisualDefinitions(createArmVisuals(lowerArmLength, lowerArmRadius, armColors.get(robotSide).brighter().brighter().brighter()));

            RigidBodyDefinition wristPitchLink = new RigidBodyDefinition(sidePrefix + "WristPitchLink");
            wristPitchLink.setMass(0.1);
            wristPitchLink.setMomentOfInertia(createNullMOI());

            RigidBodyDefinition wristRollLink = new RigidBodyDefinition(sidePrefix + "WristRollLink");
            wristRollLink.setMass(0.1);
            wristRollLink.setMomentOfInertia(createNullMOI());

            RigidBodyDefinition handLink = new RigidBodyDefinition(sidePrefix + "HandLink");
            handLink.setMass(1.0);
            handLink.setMomentOfInertia(createNullMOI());
            handLink.setVisualDefinitions(createHandVisuals());

            torsoLink.addChildJoint(shoulderYaw);
            shoulderYaw.setSuccessor(shoulderYawLink);
            shoulderYawLink.addChildJoint(shoulderRoll);
            shoulderRoll.setSuccessor(shoulderRollLink);
            shoulderRollLink.addChildJoint(shoulderPitch);
            shoulderPitch.setSuccessor(upperArmLink);
            upperArmLink.addChildJoint(elbowPitch);
            elbowPitch.setSuccessor(lowerArmLink);
            lowerArmLink.addChildJoint(wristPitch);
            wristPitch.setSuccessor(wristPitchLink);
            wristPitchLink.addChildJoint(wristRoll);
            wristRoll.setSuccessor(wristRollLink);
            wristRollLink.addChildJoint(wristYaw);
            wristYaw.setSuccessor(handLink);
         }
      }
   }

   public static Pair<FloatingJointBasics, OneDoFJointBasics[]> createInverseDynamicsRobot(RobotDefinition robotDefinition)
   {
      RigidBodyBasics rootBody = robotDefinition.newInstance(ReferenceFrame.getWorldFrame());

      FloatingJointBasics rootJoint;
      if (rootBody.getChildrenJoints().get(0) instanceof FloatingJointBasics)
         rootJoint = (FloatingJointBasics) rootBody.getChildrenJoints().get(0);
      else
         rootJoint = null;
      return new ImmutablePair<>(rootJoint, MultiBodySystemTools.filterJoints(MultiBodySystemTools.collectSubtreeJoints(rootBody), OneDoFJointBasics.class));
   }

   private static List<VisualDefinition> createTorsoVisuals(double torsoHeight, double torsoWidth, double torsoThickness, ColorDefinition color)
   {
      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.addTruncatedCone(torsoHeight, torsoThickness, 0.7 * torsoWidth, 0.7 * torsoThickness, 0.5 * torsoWidth, color);
      factory.appendTranslation(0.0, 0.0, torsoHeight);
      double eyeBigRadius = 0.35 * torsoWidth;
      factory.appendTranslation(0.0, 0.25 * torsoWidth, 1.1 * eyeBigRadius);
      factory.addEllipsoid(0.01, eyeBigRadius, eyeBigRadius, ColorDefinitions.White());
      factory.appendTranslation(0.0, -0.5 * torsoWidth, 0.0);
      factory.addEllipsoid(0.01, eyeBigRadius, eyeBigRadius, ColorDefinitions.White());
      double eyeSmallRadius = 0.3 * eyeBigRadius;
      factory.appendTranslation(0.01, 0.5 * eyeSmallRadius, -eyeSmallRadius);
      factory.addEllipsoid(0.01, eyeSmallRadius, eyeSmallRadius, ColorDefinitions.Black());
      factory.appendTranslation(0.0, -eyeSmallRadius + 0.5 * torsoWidth, 0.0);
      factory.addEllipsoid(0.01, eyeSmallRadius, eyeSmallRadius, ColorDefinitions.Black());
      return factory.getVisualDefinitions();
   }

   private static List<VisualDefinition> createHandVisuals()
   {
      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.addSphere(0.025, ColorDefinitions.Grey());
      factory.appendTranslation(0.0, 0.0, 0.05);
      factory.addEllipsoid(0.04, 0.01, 0.1);
      return factory.getVisualDefinitions();
   }

   private static List<VisualDefinition> createArmVisuals(double length, double radius, ColorDefinition color)
   {
      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.addSphere(1.2 * radius, ColorDefinitions.Grey());
      factory.appendTranslation(0, 0, 0.5 * length);
      factory.addCylinder(length, radius, color);
      return factory.getVisualDefinitions();
   }

   private static Matrix3D createNullMOI()
   {
      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setIdentity();
      momentOfInertia.scale(1.0e-4);
      return momentOfInertia;
   }
}
