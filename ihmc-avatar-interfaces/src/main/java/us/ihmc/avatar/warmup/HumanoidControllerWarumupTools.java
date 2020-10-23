package us.ihmc.avatar.warmup;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import controller_msgs.msg.dds.SO3TrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class HumanoidControllerWarumupTools
{
   public static void warmup(HumanoidControllerWarmup controllerWarmup)
   {
      PrintTools.info("Starting to warm up...");
      long startTime = System.currentTimeMillis();
      try
      {
         controllerWarmup.runWarmup();
      }
      catch (Exception e)
      {
         PrintTools.info("Warmup failed with an exception.");
      }
      double duration = 0.001 * (System.currentTimeMillis() - startTime);
      PrintTools.info("Warmup took " + duration + "s.");
   }

   public static FootstepDataListMessage createStepsInPlace(HumanoidReferenceFrames referenceFrames)
   {
      FootstepDataListMessage message = new FootstepDataListMessage();
      double z = referenceFrames.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame().getTranslationZ();
      for (RobotSide side : RobotSide.values)
      {
         FootstepDataMessage step = message.getFootstepDataList().add();
         step.setRobotSide(side.toByte());
         FramePose3D footPose = new FramePose3D(referenceFrames.getSoleFrame(side));
         footPose.changeFrame(referenceFrames.getSoleFrame(RobotSide.LEFT));
         footPose.setX(0.1);
         footPose.changeFrame(ReferenceFrame.getWorldFrame());
         double yaw = footPose.getYaw();
         footPose.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);
         footPose.setZ(z);
         step.getLocation().set(footPose.getPosition());
         step.getOrientation().set(footPose.getOrientation());
         step.setSwingDuration(0.2);
         step.setTransferDuration(0.2);
      }
      message.setFinalTransferDuration(0.2);
      return message;
   }

   public static FootTrajectoryMessage createPickUpFootMessage(RobotSide side, HumanoidReferenceFrames referenceFrames)
   {
      FootTrajectoryMessage message = new FootTrajectoryMessage();
      message.setRobotSide(side.toByte());
      FramePose3D footPose = new FramePose3D(referenceFrames.getSoleFrame(side));
      footPose.changeFrame(referenceFrames.getSoleFrame(side.getOppositeSide()));
      footPose.setX(0.0);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      double z = referenceFrames.getSoleFrame(side.getOppositeSide()).getTransformToWorldFrame().getTranslationZ();
      footPose.setZ(z + 0.05);
      SE3TrajectoryPointMessage trajectoryPoint = message.getSe3Trajectory().getTaskspaceTrajectoryPoints().add();
      trajectoryPoint.getPosition().set(footPose.getPosition());
      trajectoryPoint.getOrientation().set(footPose.getOrientation());
      trajectoryPoint.setTime(0.1);
      return message;
   }

   public static FootTrajectoryMessage createPutDownFootMessage(RobotSide side, HumanoidReferenceFrames referenceFrames)
   {
      FootTrajectoryMessage message = new FootTrajectoryMessage();
      message.setRobotSide(side.toByte());
      FramePose3D footPose = new FramePose3D(referenceFrames.getSoleFrame(side));
      footPose.changeFrame(referenceFrames.getSoleFrame(side.getOppositeSide()));
      footPose.setX(0.0);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      double z = referenceFrames.getSoleFrame(side.getOppositeSide()).getTransformToWorldFrame().getTranslationZ();
      footPose.setZ(z);
      SE3TrajectoryPointMessage trajectoryPoint = message.getSe3Trajectory().getTaskspaceTrajectoryPoints().add();
      trajectoryPoint.getPosition().set(footPose.getPosition());
      trajectoryPoint.getOrientation().set(footPose.getOrientation());
      trajectoryPoint.setTime(0.1);
      return message;
   }

   public static ChestTrajectoryMessage createChestMessage(HumanoidReferenceFrames referenceFrames)
   {
      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      Quaternion orientation = new Quaternion();
      orientation.appendYawRotation(Math.toRadians(-10.0));
      orientation.appendRollRotation(Math.toRadians(10.0));
      SO3TrajectoryMessage so3Trajectory = message.getSo3Trajectory();
      so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(referenceFrames.getPelvisZUpFrame()));
      SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
      trajectoryPoint.setTime(0.2);
      trajectoryPoint.getOrientation().set(orientation);
      trajectoryPoint.getAngularVelocity().set(new Vector3D());
      trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
      trajectoryPoint.setTime(0.4);
      trajectoryPoint.getOrientation().set(new Quaternion());
      trajectoryPoint.getAngularVelocity().set(new Vector3D());
      return message;
   }

   public static ArmTrajectoryMessage createArmMessage(FullHumanoidRobotModel fullRobotModel, RobotSide side)
   {
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics hand = fullRobotModel.getHand(side);
      OneDoFJointBasics[] joints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
      ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(side);

      for (int jointIdx = 0; jointIdx < joints.length; jointIdx++)
      {
         OneDoFJointBasics joint = joints[jointIdx];
         double angle1 = MathTools.clamp(Math.toRadians(45.0), joint.getJointLimitLower() + 0.05, joint.getJointLimitUpper() - 0.05);
         double angle2 = MathTools.clamp(0.0, joint.getJointLimitLower() + 0.05, joint.getJointLimitUpper() - 0.05);
         OneDoFJointTrajectoryMessage jointTrajectoryMessage = message.getJointspaceTrajectory().getJointTrajectoryMessages().add();
         jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(0.2, angle1, 0.0));
         jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(0.4, angle2, 0.0));
      }
      return message;
   }
}
