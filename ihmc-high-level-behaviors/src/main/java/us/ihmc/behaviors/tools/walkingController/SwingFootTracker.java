package us.ihmc.behaviors.tools.walkingController;

import controller_msgs.FootstepDataMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.robotSide.RobotSide;

public class SwingFootTracker
{
   private static final double THRESHOLD_DISTANCE_TO_LANDING = 0.08;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ControllerStatusTracker controllerStatusTracker;
   private RobotSide swingSide = RobotSide.LEFT;
   private FootstepDataMessage currentFootstep;
   private RigidBodyTransform targetFootstepPose;
   private boolean isLanding  = false;

   public SwingFootTracker(ROS2SyncedRobotModel syncedRobot, ControllerStatusTracker controllerStatusTracker)
   {
      this.syncedRobot = syncedRobot;
      this.controllerStatusTracker = controllerStatusTracker;
   }

   public void update()
   {
      if (controllerStatusTracker.isWalking())
      {
         var footsteps = controllerStatusTracker.getFootstepTracker().getFootsteps();
         if (!footsteps.isEmpty())
         {
            FootstepDataMessage newFootstep = footsteps.get(0);

            if (currentFootstep == null || !currentFootstep.equals(newFootstep))
            {
               currentFootstep = newFootstep;
               swingSide = RobotSide.fromByte(currentFootstep.getRobotSide());
               targetFootstepPose = new RigidBodyTransform(currentFootstep.getOrientation().getQuaternion(), currentFootstep.getLocation().getPoint());
               isLanding = false;
            }

            RigidBodyTransform currentFootPose  = syncedRobot.getFullRobotModel().getSoleFrame(swingSide).getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());

            Point3D currentPosition = new Point3D();
            currentPosition.set(currentFootPose.getTranslation());
            Point3D targetPosition = new Point3D();
            targetPosition.set(targetFootstepPose.getTranslation());
            double distance = currentPosition.distance(targetPosition);
            if (distance <= THRESHOLD_DISTANCE_TO_LANDING)
            {
               isLanding = true;
            }
            else
            {
               isLanding = false;
            }
         }
      }
      else
      {
         isLanding = false;
         currentFootstep = null;
      }
   }

   public RobotSide getSide()
   {
      return swingSide;
   }

   public boolean isLanding()
   {
      return isLanding;
   }
}
