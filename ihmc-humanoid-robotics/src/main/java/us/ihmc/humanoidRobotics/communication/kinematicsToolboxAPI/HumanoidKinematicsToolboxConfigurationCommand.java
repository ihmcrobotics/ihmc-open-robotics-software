package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import controller_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class HumanoidKinematicsToolboxConfigurationCommand
      implements Command<HumanoidKinematicsToolboxConfigurationCommand, HumanoidKinematicsToolboxConfigurationMessage>
{
   private long sequenceId;
   private boolean holdCurrentCenterOfMassXYPosition = true;
   private boolean enableAutoSupportPolygon = true;
   private boolean holdSupportRigidBodies = true;
   private boolean enableMultiContactSupportRegionSolver = false;

   @Override
   public void clear()
   {
      sequenceId = 0;
      holdCurrentCenterOfMassXYPosition = true;
      enableAutoSupportPolygon = true;
      holdSupportRigidBodies = true;
      enableMultiContactSupportRegionSolver = false;
   }

   @Override
   public void set(HumanoidKinematicsToolboxConfigurationCommand other)
   {
      sequenceId = other.sequenceId;
      holdCurrentCenterOfMassXYPosition = other.holdCurrentCenterOfMassXYPosition;
      enableAutoSupportPolygon = other.enableAutoSupportPolygon;
      holdSupportRigidBodies = other.holdSupportRigidBodies;
      enableMultiContactSupportRegionSolver = other.enableMultiContactSupportRegionSolver;
   }

   @Override
   public void setFromMessage(HumanoidKinematicsToolboxConfigurationMessage message)
   {
      sequenceId = message.getSequenceId();
      holdCurrentCenterOfMassXYPosition = message.getHoldCurrentCenterOfMassXyPosition();
      enableAutoSupportPolygon = message.getEnableAutoSupportPolygon();
      holdSupportRigidBodies = message.getHoldSupportRigidBodies();
      enableMultiContactSupportRegionSolver = message.getEnableMultiContactSupportRegionSolver();
   }

   public boolean holdCurrentCenterOfMassXYPosition()
   {
      return holdCurrentCenterOfMassXYPosition;
   }

   public boolean enableAutoSupportPolygon()
   {
      return enableAutoSupportPolygon;
   }

   public boolean holdSupportRigidBodies()
   {
      return holdSupportRigidBodies;
   }

   public boolean enableMultiContactSupportRegionSolver()
   {
      return enableMultiContactSupportRegionSolver;
   }

   @Override
   public Class<HumanoidKinematicsToolboxConfigurationMessage> getMessageClass()
   {
      return HumanoidKinematicsToolboxConfigurationMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
