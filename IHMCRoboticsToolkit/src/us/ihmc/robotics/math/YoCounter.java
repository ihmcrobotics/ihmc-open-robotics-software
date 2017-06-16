package us.ihmc.robotics.math;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoCounter
{
   private final YoInteger count;
   private final YoInteger maxCount;
   
   public YoCounter(String namePrefix, YoVariableRegistry registry)
   {
      count = new YoInteger(namePrefix + "Count", registry);
      maxCount = new YoInteger(namePrefix + "MaxCount", registry);
   }
   
   public void countOne()
   {
      count.add(1);
   }
   
   public void countN(int n)
   {
      count.add(n);
   }
   
   public boolean maxCountReached()
   {
      return count.getIntegerValue() >= maxCount.getIntegerValue();
   }
   
   public void resetCount()
   {
      count.set(0);
   }
   
   public void setMaxCount(int maxCount)
   {
      this.maxCount.set(maxCount);
   }
}
