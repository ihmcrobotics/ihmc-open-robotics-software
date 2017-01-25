package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;

public class SpineTrajectoryCommand implements Command<SpineTrajectoryCommand, SpineTrajectoryMessage>
{

   @Override
   public void clear()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void set(SpineTrajectoryCommand other)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void set(SpineTrajectoryMessage message)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public Class<SpineTrajectoryMessage> getMessageClass()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public boolean isCommandValid()
   {
      // TODO Auto-generated method stub
      return false;
   }

}
