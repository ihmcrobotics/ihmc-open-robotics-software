package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Trajectory3DReadOnly
{
   void reset();

   void compute(double t);

   Point3DReadOnly getPosition();

   Vector3DReadOnly getVelocity();

   Vector3DReadOnly getAcceleration();
}
