package us.ihmc.robotics.math;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

public class YoCounter
{
   private final IntegerYoVariable count;
   private final IntegerYoVariable maxCount;
   
   public YoCounter(String namePrefix, YoVariableRegistry registry)
   {
      count = new IntegerYoVariable(namePrefix + "Count", registry);
      maxCount = new IntegerYoVariable(namePrefix + "MaxCount", registry);
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
