package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;

public class SleepTask extends BehaviorAction
{
   private final SleepBehavior sleepBehavior;
   private final double sleepTime;

   public SleepTask(SleepBehavior sleepBehavior, double sleepTime)
   {
      super(sleepBehavior);
      this.sleepBehavior = sleepBehavior;
      this.sleepTime = sleepTime;
   }

   @Override
   protected void setBehaviorInput()
   {
      sleepBehavior.setSleepTime(sleepTime);
   }
}
