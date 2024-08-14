package us.ihmc.robotics.math.trajectories.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface OrientationTrajectoryGenerator extends TrajectoryGenerator
{
   Orientation3DReadOnly getOrientation();

   Vector3DReadOnly getAngularVelocity();

   Vector3DReadOnly getAngularAcceleration();

   default void getAngularData(Orientation3DBasics orientationToPack, Vector3DBasics angularVelocityToPack, Vector3DBasics angularAccelerationToPack)
   {
      orientationToPack.set(getOrientation());
      angularVelocityToPack.set(getAngularVelocity());
      angularAccelerationToPack.set(getAngularAcceleration());
   }
}