package us.ihmc.humanoidBehaviors.utilities;

import static org.junit.Assert.assertTrue;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.utilities.io.printing.PrintTools;
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

   public TimeBasedStopThreadUpdatable(RobotDataReceiver robotDataReceiver, BehaviorInterface behavior, double pauseTime, double pauseDuration, double stopTime,
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
         setRequestedBehaviorControlMode(HumanoidBehaviorControlModeEnum.PAUSE);
      }
      else if (hasThresholdBeenCrossed(resumeTime))
      {
         assertTrue(!behavior.isDone());

         PrintTools.debug(this, "Requesting Resume");
         setRequestedBehaviorControlMode(HumanoidBehaviorControlModeEnum.RESUME);
      }
      else if (hasThresholdBeenCrossed(stopTime))
      {
         PrintTools.debug(this, "Requesting Stop");
         setRequestedBehaviorControlMode(HumanoidBehaviorControlModeEnum.STOP);
      }
      else if (behavior.isDone())
      {
         doneTime = elapsedTime;
         setShouldBehaviorRunnerBeStopped(true);
      }
      else if (getRequestedBehaviorControlMode().equals(HumanoidBehaviorControlModeEnum.STOP))
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
