package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.TimeIntervalMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;

public class TimeIntervalCommand implements Command<TimeIntervalCommand, TimeIntervalMessage>
{
   private long sequenceId;
   private double startTime = Double.NaN;
   private double endTime = Double.NaN;

   public TimeIntervalCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      startTime = Double.NaN;
      endTime = Double.NaN;
   }

   @Override
   public void setFromMessage(TimeIntervalMessage message)
   {
      sequenceId = message.getSequenceId();
      startTime = message.getStartTime();
      endTime = message.getEndTime();
   }

   @Override
   public void set(TimeIntervalCommand other)
   {
      sequenceId = other.sequenceId;
      startTime = other.startTime;
      endTime = other.endTime;
   }

   public void getTimeInterval(TimeIntervalBasics timeIntervalToPack)
   {
      timeIntervalToPack.setInterval(startTime, endTime);
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

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
