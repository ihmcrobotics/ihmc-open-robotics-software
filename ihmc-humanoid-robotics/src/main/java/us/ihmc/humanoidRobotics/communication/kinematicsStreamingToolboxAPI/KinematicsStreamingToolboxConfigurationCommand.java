package us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsStreamingToolboxConfigurationCommand
      implements Command<KinematicsStreamingToolboxConfigurationCommand, KinematicsStreamingToolboxConfigurationMessage>
{
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

   private ReferenceFrame leftHandTrajectoryFrame = ReferenceFrame.getWorldFrame();
   private ReferenceFrame rightHandTrajectoryFrame = ReferenceFrame.getWorldFrame();
   private ReferenceFrame chestTrajectoryFrame = ReferenceFrame.getWorldFrame();
   private ReferenceFrame pelvisTrajectoryFrame = ReferenceFrame.getWorldFrame();

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

      leftHandTrajectoryFrame = ReferenceFrame.getWorldFrame();
      rightHandTrajectoryFrame = ReferenceFrame.getWorldFrame();
      chestTrajectoryFrame = ReferenceFrame.getWorldFrame();
      pelvisTrajectoryFrame = ReferenceFrame.getWorldFrame();
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

      leftHandTrajectoryFrame = other.leftHandTrajectoryFrame;
      rightHandTrajectoryFrame = other.rightHandTrajectoryFrame;
      chestTrajectoryFrame = other.chestTrajectoryFrame;
      pelvisTrajectoryFrame = other.pelvisTrajectoryFrame;
   }

   @Override
   public void setFromMessage(KinematicsStreamingToolboxConfigurationMessage message)
   {
      set(message, null);
   }

   public void set(KinematicsStreamingToolboxConfigurationMessage message, ReferenceFrameHashCodeResolver referenceFrameResolver)
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

      if (referenceFrameResolver != null)
      {
         leftHandTrajectoryFrame = referenceFrameResolver.getReferenceFrame(message.getLeftHandTrajectoryFrameId());
         rightHandTrajectoryFrame = referenceFrameResolver.getReferenceFrame(message.getRightHandTrajectoryFrameId());
         chestTrajectoryFrame = referenceFrameResolver.getReferenceFrame(message.getChestTrajectoryFrameId());
         pelvisTrajectoryFrame = referenceFrameResolver.getReferenceFrame(message.getPelvisTrajectoryFrameId());
      }
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

   public ReferenceFrame getHandTrajectoryFrame(RobotSide robotSide)
   {
      return robotSide == RobotSide.LEFT ? leftHandTrajectoryFrame : rightHandTrajectoryFrame;
   }
   
   public ReferenceFrame getLeftHandTrajectoryFrame()
   {
      return leftHandTrajectoryFrame;
   }

   public ReferenceFrame getRightHandTrajectoryFrame()
   {
      return rightHandTrajectoryFrame;
   }

   public ReferenceFrame getChestTrajectoryFrame()
   {
      return chestTrajectoryFrame;
   }

   public ReferenceFrame getPelvisTrajectoryFrame()
   {
      return pelvisTrajectoryFrame;
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
