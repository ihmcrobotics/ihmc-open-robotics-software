package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.NeckDesiredAccelerationsMessage;

public class NeckDesiredAccelerationsCommand extends DesiredAccelerationCommand<NeckDesiredAccelerationsCommand, NeckDesiredAccelerationsMessage>
{

   @Override
   public Class<NeckDesiredAccelerationsMessage> getMessageClass()
   {
      return NeckDesiredAccelerationsMessage.class;
   }

}
