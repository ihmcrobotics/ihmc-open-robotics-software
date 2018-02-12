package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineDesiredAccelerationsMessage;

public class SpineDesiredAccelerationCommand implements Command<SpineDesiredAccelerationCommand, SpineDesiredAccelerationsMessage>
{
   private final DesiredAccelerationCommand desiredAccelerations = new DesiredAccelerationCommand();

   @Override
   public void clear()
   {
      desiredAccelerations.clear();
   }

   @Override
   public void set(SpineDesiredAccelerationCommand other)
   {
      desiredAccelerations.set(other.desiredAccelerations);
   }

   @Override
   public void set(SpineDesiredAccelerationsMessage message)
   {
      desiredAccelerations.set(message.desiredAccelerations);
   }

   public DesiredAccelerationCommand getDesiredAccelerations()
   {
      return desiredAccelerations;
   }

   @Override
   public boolean isCommandValid()
   {
      return desiredAccelerations.isCommandValid();
   }

   @Override
   public Class<SpineDesiredAccelerationsMessage> getMessageClass()
   {
      return SpineDesiredAccelerationsMessage.class;
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return desiredAccelerations.isDelayedExecutionSupported();
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      desiredAccelerations.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      desiredAccelerations.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return desiredAccelerations.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return desiredAccelerations.getExecutionTime();
   }
}
