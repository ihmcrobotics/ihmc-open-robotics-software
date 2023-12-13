package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.CrocoddylTimeIntervalMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class CrocoddylTimeIntervalCommand implements Command<CrocoddylTimeIntervalCommand, CrocoddylTimeIntervalMessage>
{
   private int sequenceId = -1;
   private double time = Double.NaN;
   private double duration = Double.NaN;

   @Override
   public void clear()
   {
      sequenceId = -1;
      time = Double.NaN;
      duration = Double.NaN;
   }

   @Override
   public void setFromMessage(CrocoddylTimeIntervalMessage message)
   {
      this.sequenceId = (int) message.getUniqueId();
      this.time = message.getTime();
      this.duration = message.getDuration();
   }

   @Override
   public Class<CrocoddylTimeIntervalMessage> getMessageClass()
   {
      return CrocoddylTimeIntervalMessage.class;
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

   @Override
   public void set(CrocoddylTimeIntervalCommand other)
   {
      this.sequenceId = other.sequenceId;
      this.time = other.time;
      this.duration = other.duration;
   }

   public double getTime()
   {
      return time;
   }

   public double getDuration()
   {
      return duration;
   }
}
