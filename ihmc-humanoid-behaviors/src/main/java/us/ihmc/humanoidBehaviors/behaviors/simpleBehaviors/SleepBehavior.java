package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoDouble;

public class SleepBehavior extends AbstractBehavior
{
   private final YoDouble sleepTime;
   private final YoStopwatch stopwatch;

   public SleepBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime)
   {
      this(robotName, ros2Node, yoTime, 1.0);
   }

   public SleepBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime, double sleepTime)
   {
      super(robotName, ros2Node);

      this.sleepTime = new YoDouble("sleepTime", registry);
      this.sleepTime.set(sleepTime);

      stopwatch = new YoStopwatch(yoTime);
   }

   @Override
   public void doControl()
   {
   }

   public void setSleepTime(double sleepTime)
   {
      this.sleepTime.set(sleepTime);
   }

   @Override
   public void onBehaviorExited()
   {
   }

   @Override
   public boolean isDone()
   {
      return (stopwatch.totalElapsed() > sleepTime.getDoubleValue());
   }

   @Override
   public void onBehaviorEntered()
   {
      stopwatch.reset();
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
      stopwatch.suspend();
   }

   @Override
   public void onBehaviorResumed()
   {
      stopwatch.resume();
   }
}
