package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class SleepTask extends BehaviorTask
{

   public SleepTask(SleepBehavior sleepBehavior, DoubleYoVariable yoTime)
   {
      super(sleepBehavior, yoTime);
   }

   @Override
   protected void setBehaviorInput()
   {
   }
}
