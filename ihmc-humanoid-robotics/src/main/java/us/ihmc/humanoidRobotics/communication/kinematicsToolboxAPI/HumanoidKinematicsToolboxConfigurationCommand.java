package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.HumanoidKinematicsToolboxConfigurationMessage;

public class HumanoidKinematicsToolboxConfigurationCommand
      implements Command<HumanoidKinematicsToolboxConfigurationCommand, HumanoidKinematicsToolboxConfigurationMessage>
{
   private boolean holdCurrentCenterOfMassXYPosition = true;
   private boolean holdSupportFootPositions = true;

   @Override
   public void clear()
   {
      holdCurrentCenterOfMassXYPosition = true;
      holdSupportFootPositions = true;
   }

   @Override
   public void set(HumanoidKinematicsToolboxConfigurationCommand other)
   {
      holdCurrentCenterOfMassXYPosition = other.holdCurrentCenterOfMassXYPosition;
      holdSupportFootPositions = other.holdSupportFootPositions;
   }

   @Override
   public void set(HumanoidKinematicsToolboxConfigurationMessage message)
   {
      holdCurrentCenterOfMassXYPosition = message.getHoldCurrentCenterOfMassXYPosition();
      holdSupportFootPositions = message.getHoldSupportFootPositions();
   }

   public boolean holdCurrentCenterOfMassXYPosition()
   {
      return holdCurrentCenterOfMassXYPosition;
   }

   public boolean holdSupportFootPositions()
   {
      return holdSupportFootPositions;
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
}
