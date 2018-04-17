package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class MultipleWaypointsBlendedPositionTrajectoryGenerator extends BlendedPositionTrajectoryGenerator
{
   private final MultipleWaypointsPositionTrajectoryGenerator trajectory;

   public MultipleWaypointsBlendedPositionTrajectoryGenerator(String prefix, MultipleWaypointsPositionTrajectoryGenerator trajectory, ReferenceFrame trajectoryFrame,
                                                              YoVariableRegistry parentRegistry)
   {
      super(prefix, trajectory, trajectoryFrame, parentRegistry);

      this.trajectory = trajectory;
   }

   public int getCurrentPositionWaypointIndex()
   {
      return trajectory.getCurrentWaypointIndex();
   }

   public void clearTrajectory(ReferenceFrame referenceFrame)
   {
      this.trajectory.clear(referenceFrame);
   }

   public void appendPositionWaypoint(FrameEuclideanTrajectoryPoint positionWaypoint)
   {
      this.trajectory.appendWaypoint(positionWaypoint);
   }

   public void appendPositionWaypoint(double timeAtWaypoint, FramePoint3D position, FrameVector3D linearVelocity)
   {
      this.trajectory.appendWaypoint(timeAtWaypoint, position, linearVelocity);
   }
}
