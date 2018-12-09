package us.ihmc.robotics.time;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TimeIntervalCommand;
import us.ihmc.quadrupedBasics.gait.TimeIntervalBasics;
import us.ihmc.quadrupedBasics.gait.TimeIntervalReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoTimeInterval implements TimeIntervalBasics
{
   private YoDouble startTime;
   private YoDouble endTime;

   public YoTimeInterval(String prefix, YoVariableRegistry registry)
   {
      this.startTime = new YoDouble(prefix + "StartTime", registry);
      this.endTime = new YoDouble(prefix + "EndTime", registry);
   }

   @Override
   public double getStartTime()
   {
      return startTime.getDoubleValue();
   }

   @Override
   public void setStartTime(double startTime)
   {
      this.startTime.set(startTime);
   }

   @Override
   public double getEndTime()
   {
      return endTime.getDoubleValue();
   }

   @Override
   public void setEndTime(double endTime)
   {
      this.endTime.set(endTime);
   }

   @Override
   public void setInterval(double startTime, double endTime)
   {
      this.startTime.set(startTime);
      this.endTime.set(endTime);

      checkInterval();
   }

   @Override
   public TimeIntervalBasics shiftInterval(double shiftTime)
   {
      this.startTime.add(shiftTime);
      this.endTime.add(shiftTime);
      return this;
   }

   @Override
   public void set(TimeIntervalReadOnly timeInterval)
   {
      this.startTime.set(timeInterval.getStartTime());
      this.endTime.set(timeInterval.getEndTime());
   }

   @Override
   public void set(TimeIntervalCommand command)
   {
      this.startTime.set(command.getStartTime());
      this.endTime.set(command.getEndTime());
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getStringOf("(", " )", ", ", getStartTime(), getEndTime());
   }
}
