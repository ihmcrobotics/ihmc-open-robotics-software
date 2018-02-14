package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckDesiredAccelerationsMessage;

public class NeckDesiredAccelerationsCommand implements Command<NeckDesiredAccelerationsCommand, NeckDesiredAccelerationsMessage>
{
   private final DesiredAccelerationsCommand desiredAccelerations = new DesiredAccelerationsCommand();

   @Override
   public void clear()
   {
      desiredAccelerations.clear();
   }

   @Override
   public void set(NeckDesiredAccelerationsCommand other)
   {
      desiredAccelerations.set(other.desiredAccelerations);
   }

   @Override
   public void set(NeckDesiredAccelerationsMessage message)
   {
      desiredAccelerations.set(message.desiredAccelerations);
   }

   public DesiredAccelerationsCommand getDesiredAccelerations()
   {
      return desiredAccelerations;
   }

   @Override
   public boolean isCommandValid()
   {
      return desiredAccelerations.isCommandValid();
   }

   @Override
   public Class<NeckDesiredAccelerationsMessage> getMessageClass()
   {
      return NeckDesiredAccelerationsMessage.class;
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
