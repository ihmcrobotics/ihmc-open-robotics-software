package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.TimeIntervalMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class TimeIntervalCommand implements Command<TimeIntervalCommand, TimeIntervalMessage>
{
   private double startTime = Double.NaN;
   private double endTime = Double.NaN;

   public TimeIntervalCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      startTime = Double.NaN;
      endTime = Double.NaN;
   }

   @Override
   public void setFromMessage(TimeIntervalMessage message)
   {
      startTime = message.getStartTime();
      endTime = message.getEndTime();
   }

   @Override
   public void set(TimeIntervalCommand other)
   {
      startTime = other.startTime;
      endTime = other.endTime;
   }

   public double getStartTime()
   {
      return startTime;
   }

   public double getEndTime()
   {
      return endTime;
   }

   public void shiftTimeInterval(double timeToShift)
   {
      startTime += timeToShift;
      endTime += timeToShift;
   }

   @Override
   public Class<TimeIntervalMessage> getMessageClass()
   {
      return TimeIntervalMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return Double.isFinite(startTime) && Double.isFinite(endTime);
   }
}
