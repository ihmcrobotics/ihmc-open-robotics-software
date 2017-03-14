package us.ihmc.humanoidBehaviors.utilities;

import static org.junit.Assert.assertTrue;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class TimeBasedStopThreadUpdatable extends StopThreadUpdatable
{
   private final boolean DEBUG = false;

   private double startTime = Double.NaN;
   private double doneTime = Double.NaN;
   private double elapsedTime = 0.0;
   private double elapsedTimeOld = 0.0;
   private final double pauseTime;
   private final double resumeTime;
   private final double stopTime;

   public TimeBasedStopThreadUpdatable(HumanoidRobotDataReceiver robotDataReceiver, AbstractBehavior behavior, double pauseTime, double pauseDuration, double stopTime,
         ReferenceFrame frameToKeepTrackOf)
   {
      super(robotDataReceiver, behavior, frameToKeepTrackOf);

      this.pauseTime = pauseTime;
      this.resumeTime = pauseTime + pauseDuration;
      this.stopTime = stopTime;
   }

   @Override
   public void update(double time)
   {
      if (Double.isNaN(startTime))
      {
         startTime = time;
      }

      elapsedTime = time - startTime;

      if (hasThresholdBeenCrossed(pauseTime))
      {
         PrintTools.debug(this, "Requesting Pause");
         setRequestedBehaviorControlMode(BehaviorControlModeEnum.PAUSE);
      }
      else if (hasThresholdBeenCrossed(resumeTime))
      {
         assertTrue(!behavior.isDone());

         PrintTools.debug(this, "Requesting Resume");
         setRequestedBehaviorControlMode(BehaviorControlModeEnum.RESUME);
      }
      else if (hasThresholdBeenCrossed(stopTime))
      {
         PrintTools.debug(this, "Requesting Stop");
         setRequestedBehaviorControlMode(BehaviorControlModeEnum.STOP);
      }
      else if (behavior.isDone())
      {
         doneTime = elapsedTime;
         setShouldBehaviorRunnerBeStopped(true);
      }
      else if (getRequestedBehaviorControlMode().equals(BehaviorControlModeEnum.STOP))
      {
         if (Double.isNaN(doneTime))
         {
            doneTime = elapsedTime + 2.0;
         }
         else if (elapsedTime > doneTime)
         {
            setShouldBehaviorRunnerBeStopped(true);
         }
      }

      elapsedTimeOld = elapsedTime;
   }

   private boolean hasThresholdBeenCrossed(double elapsedTimeThreshold)
   {
      boolean ret = elapsedTimeOld < elapsedTimeThreshold && elapsedTime >= elapsedTimeThreshold;
      return ret;
   }
}
