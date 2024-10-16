package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;

public class KinematicsToolboxControllerTestRobotsSCS2
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

         RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");

         RigidBodyDefinition scsShoulderYawLink = new RigidBodyDefinition("shoulderYawLink");
         scsShoulderYawLink.setMass(0.1);
         scsShoulderYawLink.setMomentOfInertia(createNullMOI());

         RigidBodyDefinition scsShoulderRollLink = new RigidBodyDefinition("shoulderRollLink");
         scsShoulderRollLink.setMass(0.1);
         scsShoulderRollLink.setMomentOfInertia(createNullMOI());

         RigidBodyDefinition scsUpperArmLink = new RigidBodyDefinition("upperArmLink");
         scsUpperArmLink.setMass(1.0);
         scsUpperArmLink.setMomentOfInertia(createNullMOI());
         scsUpperArmLink.getVisualDefinitions().addAll(createArmGraphic(upperArmLength, upperArmRadius, new MaterialDefinition(ColorDefinitions.Red())));

         RigidBodyDefinition scsLowerArmLink = new RigidBodyDefinition("lowerArmLink");
         scsLowerArmLink.setMass(1.0);
         scsLowerArmLink.setMomentOfInertia(createNullMOI());
         scsLowerArmLink.getVisualDefinitions().addAll(createArmGraphic(lowerArmLength, lowerArmRadius, new MaterialDefinition(ColorDefinitions.Green())));

         RigidBodyDefinition scsWristPitchLink = new RigidBodyDefinition("wristPitchLink");
         scsWristPitchLink.setMass(0.1);
         scsWristPitchLink.setMomentOfInertia(createNullMOI());

         RigidBodyDefinition scsWristRollLink = new RigidBodyDefinition("wristRollLink");
         scsWristRollLink.setMass(0.1);
         scsWristRollLink.setMomentOfInertia(createNullMOI());

         RigidBodyDefinition scsHandLink = new RigidBodyDefinition("handLink");
         scsHandLink.setMass(1.0);
         scsHandLink.setMomentOfInertia(createNullMOI());
         scsHandLink.getVisualDefinitions().addAll(createHandGraphics());

         scsShoulderYaw.setSuccessor(scsShoulderYawLink);
         scsShoulderRoll.setSuccessor(scsShoulderRollLink);
         scsShoulderPitch.setSuccessor(scsUpperArmLink);
         scsElbowPitch.setSuccessor(scsLowerArmLink);
         scsWristPitch.setSuccessor(scsWristPitchLink);
         scsWristRoll.setSuccessor(scsWristRollLink);
         scsWristYaw.setSuccessor(scsHandLink);

         rootBody.addChildJoint(scsShoulderYaw);
         scsShoulderYawLink.addChildJoint(scsShoulderRoll);
         scsShoulderRollLink.addChildJoint(scsShoulderPitch);
         scsUpperArmLink.addChildJoint(scsElbowPitch);
         scsLowerArmLink.addChildJoint(scsWristPitch);
         scsWristPitchLink.addChildJoint(scsWristRoll);
         scsWristRollLink.addChildJoint(scsWristYaw);
         setRootBodyDefinition(rootBody);
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

      private final SideDependentList<MaterialDefinition> armAppearances = new SideDependentList<>(new MaterialDefinition(ColorDefinitions.DarkRed()),
                                                                                                   new MaterialDefinition(ColorDefinitions.DarkMagenta()));

      public UpperBodyWithTwoManipulators()
      {
         super("UpperBodyWithTwoManipulators");

         RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
         setRootBodyDefinition(rootBody);

         RevoluteJointDefinition torsoYaw = new RevoluteJointDefinition("torsoYaw", torsoYawOffset, Axis3D.Z);
         torsoYaw.setPositionLimits(-Math.PI / 3.0, Math.PI / 3.0);
         rootBody.addChildJoint(torsoYaw);

         RigidBodyDefinition torsoLink = new RigidBodyDefinition("torsoLink");
         torsoLink.setMass(5.0);
         torsoLink.setMomentOfInertia(createNullMOI());
         torsoLink.getVisualDefinitions()
                  .addAll(createTorsoGraphics(torsoHeight, torsoWidth, 0.3 * torsoWidth, new MaterialDefinition(ColorDefinitions.CadetBlue())));

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
            upperArmLink.getVisualDefinitions().addAll(createArmGraphic(upperArmLength, upperArmRadius, armAppearances.get(robotSide)));

            RigidBodyDefinition lowerArmLink = new RigidBodyDefinition(sidePrefix + "LowerArmLink");
            lowerArmLink.setMass(1.0);
            lowerArmLink.setMomentOfInertia(createNullMOI());
            lowerArmLink.getVisualDefinitions()
                        .addAll(createArmGraphic(lowerArmLength,
                                                 lowerArmRadius,
                                                 new MaterialDefinition(armAppearances.get(robotSide).getDiffuseColor().brighter().brighter())));

            RigidBodyDefinition wristPitchLink = new RigidBodyDefinition(sidePrefix + "WristPitchLink");
            wristPitchLink.setMass(0.1);
            wristPitchLink.setMomentOfInertia(createNullMOI());

            RigidBodyDefinition wristRollLink = new RigidBodyDefinition(sidePrefix + "WristRollLink");
            wristRollLink.setMass(0.1);
            wristRollLink.setMomentOfInertia(createNullMOI());

            RigidBodyDefinition handLink = new RigidBodyDefinition(sidePrefix + "HandLink");
            handLink.setMass(1.0);
            handLink.setMomentOfInertia(createNullMOI());
            handLink.getVisualDefinitions().addAll(createHandGraphics());

            shoulderYaw.setSuccessor(shoulderYawLink);
            shoulderRoll.setSuccessor(shoulderRollLink);
            shoulderPitch.setSuccessor(upperArmLink);
            elbowPitch.setSuccessor(lowerArmLink);
            wristPitch.setSuccessor(wristPitchLink);
            wristRoll.setSuccessor(wristRollLink);
            wristYaw.setSuccessor(handLink);

            torsoLink.addChildJoint(shoulderYaw);
            shoulderYawLink.addChildJoint(shoulderRoll);
            shoulderRollLink.addChildJoint(shoulderPitch);
            upperArmLink.addChildJoint(elbowPitch);
            lowerArmLink.addChildJoint(wristPitch);
            wristPitchLink.addChildJoint(wristRoll);
            wristRollLink.addChildJoint(wristYaw);
         }
      }
   }

   public static KinematicsToolboxTestRobot createInverseDynamicsRobot(RobotDefinition robotDefinition)
   {
      RigidBodyBasics rootBody = robotDefinition.newInstance(ReferenceFrame.getWorldFrame());
      return new KinematicsToolboxTestRobot(rootBody);
   }

   public static class KinematicsToolboxTestRobot
   {
      private final RigidBodyBasics rootBody;
      private final FloatingJointBasics rootJoint;
      private final OneDoFJointBasics[] oneDoFJoints;

      public KinematicsToolboxTestRobot(RigidBodyBasics rootBody)
      {
         this.rootBody = rootBody;
         if (rootBody.getChildrenJoints().get(0) instanceof FloatingJointBasics)
            rootJoint = (FloatingJointBasics) rootBody.getChildrenJoints().get(0);
         else
            rootJoint = null;
         oneDoFJoints = MultiBodySystemTools.filterJoints(MultiBodySystemTools.collectSubtreeJoints(rootBody), OneDoFJointBasics.class);
      }

      public RigidBodyBasics getRootBody()
      {
         return rootBody;
      }

      public FloatingJointBasics getRootJoint()
      {
         return rootJoint;
      }

      public OneDoFJointBasics[] getOneDoFJoints()
      {
         return oneDoFJoints;
      }

      public List<JointBasics> getAllJoints()
      {
         return SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      }
   }

   private static List<VisualDefinition> createTorsoGraphics(double torsoHeight, double torsoWidth, double torsoThickness, MaterialDefinition appearance)
   {
      VisualDefinitionFactory graphics = new VisualDefinitionFactory();
      graphics.addTruncatedCone(torsoHeight, torsoThickness, 0.7 * torsoWidth, 0.7 * torsoThickness, 0.5 * torsoWidth, appearance);
      graphics.appendTranslation(0.0, 0.0, torsoHeight);
      double eyeBigRadius = 0.35 * torsoWidth;
      graphics.appendTranslation(0.0, 0.25 * torsoWidth, 1.1 * eyeBigRadius);
      graphics.addEllipsoid(0.01, eyeBigRadius, eyeBigRadius, ColorDefinitions.White());
      graphics.appendTranslation(0.0, -0.5 * torsoWidth, 0.0);
      graphics.addEllipsoid(0.01, eyeBigRadius, eyeBigRadius, ColorDefinitions.White());
      double eyeSmallRadius = 0.3 * eyeBigRadius;
      graphics.appendTranslation(0.01, 0.5 * eyeSmallRadius, -eyeSmallRadius);
      graphics.addEllipsoid(0.01, eyeSmallRadius, eyeSmallRadius, ColorDefinitions.Black());
      graphics.appendTranslation(0.0, -eyeSmallRadius + 0.5 * torsoWidth, 0.0);
      graphics.addEllipsoid(0.01, eyeSmallRadius, eyeSmallRadius, ColorDefinitions.Black());
      return graphics.getVisualDefinitions();
   }

   private static List<VisualDefinition> createHandGraphics()
   {
      VisualDefinitionFactory graphics = new VisualDefinitionFactory();
      graphics.addSphere(0.025, ColorDefinitions.Gray());
      graphics.appendTranslation(0.0, 0.0, 0.05);
      graphics.addEllipsoid(0.04, 0.01, 0.1);
      return graphics.getVisualDefinitions();
   }

   private static List<VisualDefinition> createArmGraphic(double length, double radius, MaterialDefinition appearance)
   {
      VisualDefinitionFactory graphics = new VisualDefinitionFactory();
      graphics.addSphere(1.2 * radius, ColorDefinitions.Gray());
      graphics.appendTranslation(0.0, 0.0, 0.5 * length);
      graphics.addCylinder(length, radius, appearance);
      return graphics.getVisualDefinitions();
   }

   private static Matrix3D createNullMOI()
   {
      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setIdentity();
      momentOfInertia.scale(1.0e-4);
      return momentOfInertia;
   }
}
