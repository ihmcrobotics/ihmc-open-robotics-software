package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;

public class SleepTask<E extends Enum<E>> extends BehaviorAction<E>
{

   private final SleepBehavior sleepBehavior;
   private final double sleepTime;
   
   public SleepTask(SleepBehavior sleepBehavior, double sleepTime)
   {
     this(null, sleepBehavior, sleepTime);
   }
   
   public SleepTask(E stateEnum,SleepBehavior sleepBehavior, double sleepTime)
   {
      super(stateEnum,sleepBehavior);
      this.sleepBehavior = sleepBehavior;
      this.sleepTime = sleepTime;
   }

   @Override
   protected void setBehaviorInput()
   {
      sleepBehavior.setSleepTime(sleepTime);
   }
}
