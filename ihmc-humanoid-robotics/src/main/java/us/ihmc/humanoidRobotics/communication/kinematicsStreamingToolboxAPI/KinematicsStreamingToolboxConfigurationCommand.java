package us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class KinematicsStreamingToolboxConfigurationCommand
      implements Command<KinematicsStreamingToolboxConfigurationCommand, KinematicsStreamingToolboxConfigurationMessage>
{
   public static final long WORLD_FRAME_ID = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
   private long sequenceId = -1;

   private boolean lockPelvis = false;
   private boolean lockChest = false;

   private boolean enableLeftArmJointspace = true;
   private boolean enableRightArmJointspace = true;
   private boolean enableNeckJointspace = true;

   private boolean enableLeftHandTaskspace = true;
   private boolean enableRightHandTaskspace = true;
   private boolean enableChestTaskspace = true;
   private boolean enablePelvisTaskspace = true;

   private long leftHandTrajectoryFrameId = WORLD_FRAME_ID;
   private long rightHandTrajectoryFrameId = WORLD_FRAME_ID;
   private long chestTrajectoryFrameId = WORLD_FRAME_ID;
   private long pelvisTrajectoryFrameId = WORLD_FRAME_ID;

   public KinematicsStreamingToolboxConfigurationCommand()
   {
   }

   @Override
   public void clear()
   {
      sequenceId = -1;

      lockPelvis = false;
      lockChest = false;
      enableLeftArmJointspace = true;
      enableRightArmJointspace = true;
      enableNeckJointspace = true;

      enableLeftHandTaskspace = true;
      enableRightHandTaskspace = true;
      enableChestTaskspace = true;
      enablePelvisTaskspace = true;

      leftHandTrajectoryFrameId = WORLD_FRAME_ID;
      rightHandTrajectoryFrameId = WORLD_FRAME_ID;
      chestTrajectoryFrameId = WORLD_FRAME_ID;
      pelvisTrajectoryFrameId = WORLD_FRAME_ID;
   }

   @Override
   public void set(KinematicsStreamingToolboxConfigurationCommand other)
   {
      sequenceId = other.sequenceId;

      lockPelvis = other.lockPelvis;
      lockChest = other.lockChest;
      enableLeftArmJointspace = other.enableLeftArmJointspace;
      enableRightArmJointspace = other.enableRightArmJointspace;
      enableNeckJointspace = other.enableNeckJointspace;

      enableLeftHandTaskspace = other.enableLeftHandTaskspace;
      enableRightHandTaskspace = other.enableRightHandTaskspace;
      enableChestTaskspace = other.enableChestTaskspace;
      enablePelvisTaskspace = other.enablePelvisTaskspace;

      leftHandTrajectoryFrameId = other.leftHandTrajectoryFrameId;
      rightHandTrajectoryFrameId = other.rightHandTrajectoryFrameId;
      chestTrajectoryFrameId = other.chestTrajectoryFrameId;
      pelvisTrajectoryFrameId = other.pelvisTrajectoryFrameId;
   }

   @Override
   public void setFromMessage(KinematicsStreamingToolboxConfigurationMessage message)
   {
      clear();

      sequenceId = message.getSequenceId();

      lockPelvis = message.getLockPelvis();
      lockChest = message.getLockChest();

      enableLeftArmJointspace = message.getEnableLeftArmJointspace();
      enableRightArmJointspace = message.getEnableRightArmJointspace();
      enableNeckJointspace = message.getEnableNeckJointspace();

      enableLeftHandTaskspace = message.getEnableLeftHandTaskspace();
      enableRightHandTaskspace = message.getEnableRightHandTaskspace();
      enableChestTaskspace = message.getEnableChestTaskspace();
      enablePelvisTaskspace = message.getEnablePelvisTaskspace();

      leftHandTrajectoryFrameId = message.getLeftHandTrajectoryFrameId();
      rightHandTrajectoryFrameId = message.getRightHandTrajectoryFrameId();
      chestTrajectoryFrameId = message.getChestTrajectoryFrameId();
      pelvisTrajectoryFrameId = message.getPelvisTrajectoryFrameId();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   public boolean isLockPelvis()
   {
      return lockPelvis;
   }

   public boolean isLockChest()
   {
      return lockChest;
   }

   public boolean isArmJointspaceEnabled(RobotSide robotSide)
   {
      return robotSide == RobotSide.LEFT ? isLeftArmJointspaceEnabled() : isRightArmJointspaceEnabled();
   }

   public boolean isLeftArmJointspaceEnabled()
   {
      return enableLeftArmJointspace;
   }

   public boolean isRightArmJointspaceEnabled()
   {
      return enableRightArmJointspace;
   }

   public boolean isNeckJointspaceEnabled()
   {
      return enableNeckJointspace;
   }

   public boolean isHandTaskspaceEnabled(RobotSide robotSide)
   {
      return robotSide == RobotSide.LEFT ? isLeftHandTaskspaceEnabled() : isRightHandTaskspaceEnabled();
   }

   public boolean isLeftHandTaskspaceEnabled()
   {
      return enableLeftHandTaskspace;
   }

   public boolean isRightHandTaskspaceEnabled()
   {
      return enableRightHandTaskspace;
   }

   public boolean isChestTaskspaceEnabled()
   {
      return enableChestTaskspace;
   }

   public boolean isPelvisTaskspaceEnabled()
   {
      return enablePelvisTaskspace;
   }

   public long getHandTrajectoryFrameId(RobotSide robotSide)
   {
      return robotSide == RobotSide.LEFT ? leftHandTrajectoryFrameId : rightHandTrajectoryFrameId;
   }

   public long getLeftHandTrajectoryFrameId()
   {
      return leftHandTrajectoryFrameId;
   }

   public long getRightHandTrajectoryFrameId()
   {
      return rightHandTrajectoryFrameId;
   }

   public long getChestTrajectoryFrameId()
   {
      return chestTrajectoryFrameId;
   }

   public long getPelvisTrajectoryFrameId()
   {
      return pelvisTrajectoryFrameId;
   }

   @Override
   public Class<KinematicsStreamingToolboxConfigurationMessage> getMessageClass()
   {
      return KinematicsStreamingToolboxConfigurationMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }
}
