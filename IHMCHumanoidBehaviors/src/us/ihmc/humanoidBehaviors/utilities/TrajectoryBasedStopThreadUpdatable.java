package us.ihmc.humanoidBehaviors.utilities;

import static org.junit.Assert.assertTrue;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class TrajectoryBasedStopThreadUpdatable extends StopThreadUpdatable
{
   private final boolean DEBUG = false;

   private final FramePose initialPose;
   private final FramePose currentPose;
   private final FramePose poseAtTrajectoryEnd;
   private boolean initialPoseHasBeenSet = false;

   private double trajectoryLength;

   private double startTime = Double.NaN;
   private double doneTime = Double.NaN;
   private double percentTrajectoryCompleted = 0.0;
   private double percentTrajectoryCompletedOld = 0.0;
   private final double pausePercent;
   private final double resumePercent;
   private final double stopPercent;

   public TrajectoryBasedStopThreadUpdatable(RobotDataReceiver robotDataReceiver, BehaviorInterface behavior, double pausePercent, double resumePercent,
         double stopPercent, FramePose poseAtTrajectoryEnd, ReferenceFrame frameToKeepTrackOf)
   {
      super(robotDataReceiver, behavior, frameToKeepTrackOf);

      this.initialPose = new FramePose();
      this.currentPose = new FramePose();
      this.poseAtTrajectoryEnd = poseAtTrajectoryEnd;
      
      this.pausePercent = pausePercent;
      this.resumePercent = resumePercent;
      this.stopPercent = stopPercent;
   }

   @Override
   public void update(double time)
   {
      if (Double.isNaN(startTime))
      {
         startTime = time;
      }
      double elapsedTime = time - startTime;

      if (!initialPoseHasBeenSet)
      {
         getCurrentFramePose(initialPose);

         this.trajectoryLength = initialPose.getPositionDistance(poseAtTrajectoryEnd);
         initialPoseHasBeenSet = true;
      }
      
      getCurrentFramePose(currentPose);
      double trajectoryLengthCompleted = initialPose.getPositionDistance(currentPose);
      percentTrajectoryCompleted = 100.0 * trajectoryLengthCompleted / trajectoryLength;

      if (percentTrajectoryCompletedOld < pausePercent && percentTrajectoryCompleted >= pausePercent)
      {
         SysoutTool.println("Requesting Pause", DEBUG);
         setRequestedBehaviorControlMode(HumanoidBehaviorControlModeEnum.PAUSE);
      }
      else if (percentTrajectoryCompletedOld < resumePercent && percentTrajectoryCompleted >= resumePercent)
      {
         assertTrue(!behavior.isDone());

         SysoutTool.println("Requesting Resume", DEBUG);
         setRequestedBehaviorControlMode(HumanoidBehaviorControlModeEnum.RESUME);
      }
      else if (percentTrajectoryCompletedOld < stopPercent && percentTrajectoryCompleted >= stopPercent)
      {
         SysoutTool.println("Requesting Stop", DEBUG);
         setRequestedBehaviorControlMode(HumanoidBehaviorControlModeEnum.STOP);
      }
      else if (behavior.isDone())
      {
         doneTime = time;
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
      percentTrajectoryCompletedOld = percentTrajectoryCompleted;
   }

   public double getPercentTrajectoryCompleted()
   {
      return percentTrajectoryCompleted;
   }

   protected void getCurrentFramePose(FramePose poseToPack)
   {
      poseToPack.setPoseIncludingFrame(worldFrame, frameToKeepTrackOf.getTransformToWorldFrame());
   }
}
