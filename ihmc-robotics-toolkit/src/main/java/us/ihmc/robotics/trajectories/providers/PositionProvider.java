package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface PositionProvider
{
   Point3DReadOnly getPosition();
}
