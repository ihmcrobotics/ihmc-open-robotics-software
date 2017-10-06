package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.robotics.geometry.interfaces.OneDoFWaypointInterface;

public interface OneDoFTrajectoryPointInterface<T extends OneDoFTrajectoryPointInterface<T>> extends TrajectoryPointInterface<T>, OneDoFWaypointInterface<T>
{
}
