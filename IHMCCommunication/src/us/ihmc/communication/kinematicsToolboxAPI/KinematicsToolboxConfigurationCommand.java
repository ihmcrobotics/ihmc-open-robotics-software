package us.ihmc.communication.kinematicsToolboxAPI;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.KinematicsToolboxConfigurationMessage;

public class KinematicsToolboxConfigurationCommand implements Command<KinematicsToolboxConfigurationCommand, KinematicsToolboxConfigurationMessage>
{
   private boolean holdCurrentCenterOfMassXYPosition = true;
   private boolean holdSupporFootPositions = true;


   @Override
   public void clear()
   {
      holdCurrentCenterOfMassXYPosition = true;
      holdSupporFootPositions = true;
   }

   @Override
   public void set(KinematicsToolboxConfigurationCommand other)
   {
      holdCurrentCenterOfMassXYPosition = other.holdCurrentCenterOfMassXYPosition;
      holdSupporFootPositions = other.holdSupporFootPositions;
   }

   @Override
   public void set(KinematicsToolboxConfigurationMessage message)
   {
      holdCurrentCenterOfMassXYPosition = message.holdCurrentCenterOfMassXYPosition();
      holdSupporFootPositions = message.holdSupporFootPositions();
   }

   public boolean holdCurrentCenterOfMassXYPosition()
   {
      return holdCurrentCenterOfMassXYPosition;
   }

   public boolean holdSupporFootPositions()
   {
      return holdSupporFootPositions;
   }

   @Override
   public Class<KinematicsToolboxConfigurationMessage> getMessageClass()
   {
      return KinematicsToolboxConfigurationMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }
}
