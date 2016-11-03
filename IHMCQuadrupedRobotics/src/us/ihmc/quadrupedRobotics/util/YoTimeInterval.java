package us.ihmc.quadrupedRobotics.util;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class YoTimeInterval extends TimeInterval
{
   private DoubleYoVariable startTime;
   private DoubleYoVariable endTime;

   public YoTimeInterval(String prefix, YoVariableRegistry registry)
   {
      this.startTime = new DoubleYoVariable(prefix + "StartTime", registry);
      this.endTime = new DoubleYoVariable(prefix + "EndTime", registry);
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
}
