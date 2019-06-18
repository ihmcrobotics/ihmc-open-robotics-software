package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.SpineDesiredAccelerationsMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class SpineDesiredAccelerationsCommand implements Command<SpineDesiredAccelerationsCommand, SpineDesiredAccelerationsMessage>
{
   private long sequenceId;
   private final DesiredAccelerationsCommand desiredAccelerations = new DesiredAccelerationsCommand();

   @Override
   public void clear()
   {
      sequenceId = 0;
      desiredAccelerations.clear();
   }

   @Override
   public void set(SpineDesiredAccelerationsCommand other)
   {
      sequenceId = other.sequenceId;
      desiredAccelerations.set(other.desiredAccelerations);
   }

   @Override
   public void setFromMessage(SpineDesiredAccelerationsMessage message)
   {
      sequenceId = message.getSequenceId();
      desiredAccelerations.setFromMessage(message.getDesiredAccelerations());
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

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
