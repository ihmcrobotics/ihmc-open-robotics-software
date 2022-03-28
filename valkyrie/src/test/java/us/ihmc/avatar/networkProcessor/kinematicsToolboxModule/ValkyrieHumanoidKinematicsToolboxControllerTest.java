package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieHumanoidKinematicsToolboxControllerTest extends HumanoidKinematicsToolboxControllerTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
   private final DRCRobotModel ghostRobotModel = new ValkyrieRobotModel(RobotTarget.SCS);

   @Override
   @Test // (timeout = 30000)
   public void testHoldBodyPose() throws Exception
   {
      super.testHoldBodyPose();
   }

   @Override
   @Test // (timeout = 30000)
   public void testRandomHandPoses() throws Exception
   {
      super.testRandomHandPoses();
   }

   @Override
   @Test // (timeout = 30000)
   public void testRandomHandPositions() throws Exception
   {
      super.testRandomHandPositions();
   }

   @Override
   @Test // (timeout = 30000)
   public void testSingleSupport() throws Exception
   {
      super.testSingleSupport();
   }

   @Override
   @Test
   public void testCenterOfMassConstraint() throws Exception
   {
      super.testCenterOfMassConstraint();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Override
   protected HumanoidKinematicsToolboxControllerTest.MultiContactConstraintData createMultiContactConstraintData()
   {
      MultiContactConstraintData contactConstraintData = new MultiContactConstraintData();
      contactConstraintData.footPoses.get(RobotSide.LEFT).getPosition().set(-0.7, 0.35, -0.15);
      contactConstraintData.footPoses.get(RobotSide.RIGHT).getPosition().set(-0.55, -0.35, -0.45);
      contactConstraintData.handPoses.get(RobotSide.LEFT).getPosition().set(0.7, 0.45, 0.25);
      contactConstraintData.handPoses.get(RobotSide.RIGHT).getPosition().set(0.5, -0.5, -0.05);

      contactConstraintData.footPoses.get(RobotSide.LEFT).getOrientation().setYawPitchRoll(0.0, Math.toRadians(60.0), Math.toRadians(20.0));
      contactConstraintData.footPoses.get(RobotSide.RIGHT).getOrientation().setYawPitchRoll(0.0, Math.toRadians(40.0), -Math.toRadians(20.0));
      contactConstraintData.handPoses.get(RobotSide.LEFT).getOrientation().setYawPitchRoll(0.0, Math.toRadians(0.0), -Math.toRadians(10.0));
      contactConstraintData.handPoses.get(RobotSide.RIGHT).getOrientation().setYawPitchRoll(0.0, Math.toRadians(0.0), Math.toRadians(10.0));

      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", ReferenceFrame.getWorldFrame());

      poseReferenceFrame.setPoseAndUpdate(contactConstraintData.footPoses.get(RobotSide.LEFT));
      contactConstraintData.footNormals.get(RobotSide.LEFT).setIncludingFrame(poseReferenceFrame, Axis3D.Z);
      contactConstraintData.footNormals.get(RobotSide.LEFT).changeFrame(ReferenceFrame.getWorldFrame());

      poseReferenceFrame.setPoseAndUpdate(contactConstraintData.footPoses.get(RobotSide.RIGHT));
      contactConstraintData.footNormals.get(RobotSide.RIGHT).setIncludingFrame(poseReferenceFrame, Axis3D.Z);
      contactConstraintData.footNormals.get(RobotSide.RIGHT).changeFrame(ReferenceFrame.getWorldFrame());

      poseReferenceFrame.setPoseAndUpdate(contactConstraintData.handPoses.get(RobotSide.LEFT));
      contactConstraintData.handNormals.get(RobotSide.LEFT).setIncludingFrame(poseReferenceFrame, Axis3D.Z);
      contactConstraintData.handNormals.get(RobotSide.LEFT).changeFrame(ReferenceFrame.getWorldFrame());

      poseReferenceFrame.setPoseAndUpdate(contactConstraintData.handPoses.get(RobotSide.RIGHT));
      contactConstraintData.handNormals.get(RobotSide.RIGHT).setIncludingFrame(poseReferenceFrame, Axis3D.Z);
      contactConstraintData.handNormals.get(RobotSide.RIGHT).changeFrame(ReferenceFrame.getWorldFrame());

      contactConstraintData.nominalCenterOfMass.set(0.0, -0.02, 0.19);
      contactConstraintData.centerOfMassSampleWindowX = 0.07;
      contactConstraintData.centerOfMassSampleWindowY = 0.13;

      contactConstraintData.initialConfigurationSetup = fullRobotModel ->
      {
         fullRobotModel.getRootJoint().getJointPose().getOrientation().setYawPitchRoll(0.0, 1.4, 0.0);
         fullRobotModel.getRootJoint().getJointPose().getPosition().set(-0.1, 0.0, 0.2);

         RigidBodyBasics chest = fullRobotModel.getChest();

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
            OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
            for (OneDoFJointBasics joint : armJoints)
            {
               joint.setQ(0.5 * (joint.getJointLimitLower() + joint.getJointLimitUpper()));
            }

            fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH).setQ(1.5);
            fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH).setQ(-1.6);
         }

         fullRobotModel.updateFrames();
      };

      return contactConstraintData;
   }

   @Override
   public DRCRobotModel getGhostRobotModel()
   {
      return ghostRobotModel;
   }
}
