package us.ihmc.quadrupedRobotics.providers;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public interface QuadrupedPostureInputProviderInterface
{
   Point3D getComPositionInput();

   Vector3D getComVelocityInput();

   Quaternion getBodyOrientationInput();

   Vector3D getBodyAngularRateInput();
}