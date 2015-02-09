package us.ihmc.humanoidBehaviors.utilities;

import static org.junit.Assert.assertTrue;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class TimeBasedStopThreadUpdatable extends StopThreadUpdatable
{
   private final boolean DEBUG = false;

   private double startTime = Double.NaN;
   private double doneTime = Double.NaN;
   private double elapsedTimeOld = 0.0;
   private final double pauseTime;
   private final double resumeTime;
   private final double stopTime;

   public TimeBasedStopThreadUpdatable(RobotDataReceiver robotDataReceiver, BehaviorInterface behavior, double pauseTime, double resumeTime, double stopTime,
         ReferenceFrame frameToKeepTrackOf)
   {
      super(robotDataReceiver, behavior, frameToKeepTrackOf);

      this.pauseTime = pauseTime;
      this.resumeTime = resumeTime;
      this.stopTime = stopTime;
   }

   @Override
   public void update(double time)
   {
      if (Double.isNaN(startTime))
      {
         startTime = time;
      }

      double elapsedTime = time - startTime;

      if (elapsedTimeOld < pauseTime && elapsedTime >= pauseTime)
      {
         SysoutTool.println("Requesting Pause", DEBUG);
         setRequestedBehaviorControlMode(HumanoidBehaviorControlModeEnum.PAUSE);
      }
      else if (elapsedTimeOld < resumeTime && elapsedTime >= resumeTime)
      {
         assertTrue(!behavior.isDone());

         SysoutTool.println("Requesting Resume", DEBUG);
         setRequestedBehaviorControlMode(HumanoidBehaviorControlModeEnum.RESUME);
      }
      else if (elapsedTimeOld < stopTime && elapsedTime >= stopTime)
      {
         SysoutTool.println("Requesting Stop", DEBUG);
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
}
