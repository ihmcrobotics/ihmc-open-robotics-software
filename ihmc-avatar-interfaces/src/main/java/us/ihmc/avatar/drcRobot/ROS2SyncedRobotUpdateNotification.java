package us.ihmc.avatar.drcRobot;

import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.FramePose3D;

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

   private final FramePose3D previousPelvisPose = new FramePose3D();
   private final FramePose3D nextPelvisPose = new FramePose3D();

   private final Notification notification = new Notification();

   public ROS2SyncedRobotUpdateNotification()
   {
      previousPelvisPose.setToNaN();
      nextPelvisPose.setToNaN();
   }

   public void update(ROS2SyncedRobotModel syncedRobot)
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
               nextPelvisPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());

               if (!previousPelvisPose.containsNaN())
               {
                  if (!nextPelvisPose.geometricallyEquals(previousPelvisPose, 1e-11))
                  {
                     notification.set();
                  }
               }

               previousPelvisPose.set(nextPelvisPose);
            }
         }
         else
         {
            previousPelvisPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
         }

         previousMonotonicTimeNanos = nextMonotonicTimeNanos;
      }
   }

   public Notification getNotification()
   {
      return notification;
   }
}
