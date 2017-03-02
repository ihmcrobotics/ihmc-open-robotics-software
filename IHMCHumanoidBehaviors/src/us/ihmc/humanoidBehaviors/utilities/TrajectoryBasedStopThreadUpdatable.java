package us.ihmc.humanoidBehaviors.utilities;

import static org.junit.Assert.assertTrue;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.io.printing.PrintTools;

public class TrajectoryBasedStopThreadUpdatable extends StopThreadUpdatable
{
   private final boolean DEBUG = false;

   private final FramePose initialPose;
   private final FramePose currentPose;
   private final FramePose poseAtTrajectoryEnd;
   private boolean initialPoseHasBeenSet = false;

   private double trajectoryLength;

   private double elapsedTime = 0.0;
   private double elapsedTimeOld = 0.0;
   private double startTime = Double.NaN;
   private double pauseStartTime = Double.NaN;
   private double doneTime = Double.NaN;
   private double percentTrajectoryCompleted = 0.0;
   private double percentTrajectoryCompletedOld = 0.0;
   private final double pausePercent;
   private final double pauseDuration;
   private final double stopPercent;

   public TrajectoryBasedStopThreadUpdatable(HumanoidRobotDataReceiver robotDataReceiver, AbstractBehavior behavior, double pausePercent, double pauseDuration,
         double stopPercent, FramePose2d pose2dAtTrajectoryEnd, ReferenceFrame frameToKeepTrackOf)
   {
      this(robotDataReceiver, behavior, pausePercent, pauseDuration, stopPercent, new FramePose(), frameToKeepTrackOf);

      RigidBodyTransform transformToWorldAtTrajectoryEnd = new RigidBodyTransform();
      pose2dAtTrajectoryEnd.getPose(transformToWorldAtTrajectoryEnd);
      poseAtTrajectoryEnd.setPoseIncludingFrame(worldFrame, transformToWorldAtTrajectoryEnd);
   }

   public TrajectoryBasedStopThreadUpdatable(HumanoidRobotDataReceiver robotDataReceiver, AbstractBehavior behavior, double pausePercent, double pauseDuration,
         double stopPercent, FramePose poseAtTrajectoryEnd, ReferenceFrame frameToKeepTrackOf)
   {
      super(robotDataReceiver, behavior, frameToKeepTrackOf);

      this.initialPose = new FramePose();
      this.currentPose = new FramePose();
      this.poseAtTrajectoryEnd = poseAtTrajectoryEnd;

      this.pausePercent = pausePercent;
      this.pauseDuration = pauseDuration;
      this.stopPercent = stopPercent;
   }

   @Override
   public void update(double time)
   {
      if (Double.isNaN(startTime))
      {
         startTime = time;
      }
      elapsedTime = time - startTime;

      if (!initialPoseHasBeenSet)
      {
         getCurrentTestFramePose(initialPose);

         this.trajectoryLength = initialPose.getPositionDistance(poseAtTrajectoryEnd);
         initialPoseHasBeenSet = true;
      }

      getCurrentTestFramePose(currentPose);
      double trajectoryLengthCompleted = initialPose.getPositionDistance(currentPose);
      percentTrajectoryCompleted = 100.0 * trajectoryLengthCompleted / trajectoryLength;

      if (hasThresholdBeenCrossed(pausePercent))
      {
         PrintTools.debug(this, "Requesting Pause");
         setRequestedBehaviorControlMode(BehaviorControlModeEnum.PAUSE);
         pauseStartTime = elapsedTime;
      }
      else if ((elapsedTimeOld - pauseStartTime) < pauseDuration && (elapsedTime - pauseStartTime) >= pauseDuration)
      {
         assertTrue(!behavior.isDone());

         PrintTools.debug(this, "Requesting Resume");
         setRequestedBehaviorControlMode(BehaviorControlModeEnum.RESUME);
      }
      else if (hasThresholdBeenCrossed(stopPercent))
      {
         PrintTools.debug(this, "Requesting Stop");
         setRequestedBehaviorControlMode(BehaviorControlModeEnum.STOP);
      }
      else if (behavior.isDone())
      {
         doneTime = time;
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
      percentTrajectoryCompletedOld = percentTrajectoryCompleted;
   }

   private boolean hasThresholdBeenCrossed(double percentageThreshold)
   {
      boolean ret = percentTrajectoryCompletedOld < percentageThreshold && percentTrajectoryCompleted >= percentageThreshold;
      return ret;
   }

   public double getPercentTrajectoryCompleted()
   {
      return percentTrajectoryCompleted;
   }
}
