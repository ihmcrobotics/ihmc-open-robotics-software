package us.ihmc.avatar.drcRobot;

import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * This class is necessary because in some situations,
 * RobotConfigurationData can:
 * <ul>
 *    <li>Have not recieved a new message</li>
 *    <li>Monotonic time might not have changed</li>
 *    <li>The poses of the robot might not have been updated</li>
 * </ul>
 *
 * These things can happen for a lot of different reasons. To be
 * sure we recieved a full and meaningful update, we use this class
 * when we need to be sure, such as for calculating tracking
 * derivatives.
 */
public class ROS2SyncedRobotUpdateNotification
{
   private long syncedRobotUpdateNumber = -1;
   private long previousMonotonicTimeNanos = -1;
   private long nextMonotonicTimeNanos = -1;

   private final FramePose3D previousPose = new FramePose3D();
   private final FramePose3D nextPose = new FramePose3D();

   private final Notification notification = new Notification();

   public ROS2SyncedRobotUpdateNotification()
   {
      previousPose.setToNaN();
      nextPose.setToNaN();
   }

   public void update(ROS2SyncedRobotModel syncedRobot, ReferenceFrame referenceFrameThatShouldBeMoving)
   {
      if (syncedRobot.getLatestRobotConfigurationData().getSequenceId() > syncedRobotUpdateNumber)
      {
         syncedRobotUpdateNumber = syncedRobot.getLatestRobotConfigurationData().getSequenceId();

         boolean pastFirstUpdate = previousMonotonicTimeNanos > -1;

         nextMonotonicTimeNanos = syncedRobot.getLatestRobotConfigurationData().getMonotonicTime();

         if (pastFirstUpdate)
         {
            double deltaMonotonicTimeNanos = nextMonotonicTimeNanos - previousMonotonicTimeNanos;
            boolean deltaTimeIsNonZero = deltaMonotonicTimeNanos > 0;

            if (deltaTimeIsNonZero)
            {
               nextPose.setFromReferenceFrame(referenceFrameThatShouldBeMoving);

               if (!previousPose.containsNaN())
               {
                  if (!nextPose.geometricallyEquals(previousPose, 1e-11))
                  {
                     notification.set();
                  }
               }

               previousPose.set(nextPose);
            }
         }
         else
         {
            previousPose.setFromReferenceFrame(referenceFrameThatShouldBeMoving);
         }

         previousMonotonicTimeNanos = nextMonotonicTimeNanos;
      }
   }

   public Notification getNotification()
   {
      return notification;
   }
}
