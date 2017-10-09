package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.SpineDesiredAccelerationsMessage;

public class SpineDesiredAccelerationCommand extends DesiredAccelerationCommand<SpineDesiredAccelerationCommand, SpineDesiredAccelerationsMessage>
{

   @Override
   public Class<SpineDesiredAccelerationsMessage> getMessageClass()
   {
      return SpineDesiredAccelerationsMessage.class;
   }

}
