package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.utilities.taskExecutor.Task;

public abstract class BehaviorTask implements Task
{
   private final BehaviorInterface behavior;

   protected final DoubleYoVariable yoTime;
   protected double behaviorDoneTime = Double.NaN;
   protected final double sleepTime;

   public BehaviorTask(BehaviorInterface behavior, DoubleYoVariable yoTime)
   {
      this(behavior, yoTime, 0.0);
   }

   public BehaviorTask(BehaviorInterface behavior, DoubleYoVariable yoTime, double sleepTime)
   {
      this.behavior = behavior;
      this.yoTime = yoTime;
      this.sleepTime = sleepTime;
   }

   @Override
   public void doTransitionIntoAction()
   {
      behavior.initialize();
      setBehaviorInput();
   }

   protected abstract void setBehaviorInput();

   @Override
   public void doAction()
   {
      behavior.doControl();
      if (Double.isNaN(behaviorDoneTime) && behavior.isDone())
      {
         behaviorDoneTime = yoTime.getDoubleValue();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      behavior.doPostBehaviorCleanup();
   }

   @Override
   public boolean isDone()
   {
      boolean sleepTimeAchieved = yoTime.getDoubleValue() > behaviorDoneTime + sleepTime;
      return behavior.isDone() && sleepTimeAchieved;
   }

   @Override
   public void pause()
   {
      behavior.pause();
   }

   @Override
   public void resume()
   {
      behavior.resume();
   }

   @Override
   public void stop()
   {
      behavior.stop();
   }

   public BehaviorInterface getBehavior()
   {
      return behavior;
   }
}
