package us.ihmc.quadrupedRobotics.util;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoTimeInterval extends TimeInterval
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
}
