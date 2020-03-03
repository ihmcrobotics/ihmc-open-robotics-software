package us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI;

import controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.robotSide.RobotSide;

public class KinematicsStreamingToolboxOutputConfigurationCommand
      implements Command<KinematicsStreamingToolboxOutputConfigurationCommand, KinematicsStreamingToolboxOutputConfigurationMessage>
{
   private long sequenceId = -1;

   private boolean enableLeftArmJointspace = true;
   private boolean enableRightArmJointspace = true;
   private boolean enableNeckJointspace = true;

   private boolean enableLeftHandTaskspace = true;
   private boolean enableRightHandTaskspace = true;
   private boolean enableChestTaskspace = true;
   private boolean enablePelvisTaskspace = true;

   public KinematicsStreamingToolboxOutputConfigurationCommand()
   {
   }

   @Override
   public void clear()
   {
      sequenceId = -1;

      enableLeftArmJointspace = true;
      enableRightArmJointspace = true;
      enableNeckJointspace = true;

      enableLeftHandTaskspace = true;
      enableRightHandTaskspace = true;
      enableChestTaskspace = true;
      enablePelvisTaskspace = true;
   }

   @Override
   public void set(KinematicsStreamingToolboxOutputConfigurationCommand other)
   {
      sequenceId = other.sequenceId;

      enableLeftArmJointspace = other.enableLeftArmJointspace;
      enableRightArmJointspace = other.enableRightArmJointspace;
      enableNeckJointspace = other.enableNeckJointspace;

      enableLeftHandTaskspace = other.enableLeftHandTaskspace;
      enableRightHandTaskspace = other.enableRightHandTaskspace;
      enableChestTaskspace = other.enableChestTaskspace;
      enablePelvisTaskspace = other.enablePelvisTaskspace;
   }

   @Override
   public void setFromMessage(KinematicsStreamingToolboxOutputConfigurationMessage message)
   {
      sequenceId = message.getSequenceId();

      enableLeftArmJointspace = message.getEnableLeftArmJointspace();
      enableRightArmJointspace = message.getEnableRightArmJointspace();
      enableNeckJointspace = message.getEnableNeckJointspace();

      enableLeftHandTaskspace = message.getEnableLeftHandTaskspace();
      enableRightHandTaskspace = message.getEnableRightHandTaskspace();
      enableChestTaskspace = message.getEnableChestTaskspace();
      enablePelvisTaskspace = message.getEnablePelvisTaskspace();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
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

   @Override
   public Class<KinematicsStreamingToolboxOutputConfigurationMessage> getMessageClass()
   {
      return KinematicsStreamingToolboxOutputConfigurationMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }
}
