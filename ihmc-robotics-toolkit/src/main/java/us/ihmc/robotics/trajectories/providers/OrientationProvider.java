package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;

public interface OrientationProvider
{
   Orientation3DReadOnly getOrientation();
}
