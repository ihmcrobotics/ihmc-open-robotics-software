package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;

/**
 * This interface is used for the swing trajectory generation. It allows convenient switching between planning
 * swing trajectories in 3d for custom waypoints or 2d for faster planning with default waypoints.
 */
public interface WaypointTrajectoryGenerator extends PositionTrajectoryGenerator
{
   public void setEndpointConditions(FramePoint initialPosition, FrameVector initialVelocity, FramePoint finalPosition, FrameVector finalVelocity);

   public void setWaypoints(ArrayList<FramePoint> waypointPositions);

   public void informDone();

   public double getMaxSpeed();

   public void computeMaxSpeed();
}
