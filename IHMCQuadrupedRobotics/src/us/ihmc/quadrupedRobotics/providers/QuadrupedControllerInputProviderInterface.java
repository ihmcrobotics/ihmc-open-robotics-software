package us.ihmc.quadrupedRobotics.providers;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public interface QuadrupedControllerInputProviderInterface
{

   Point3d getComPositionInput();

   Vector3d getComVelocityInput();

   Quat4d getBodyOrientationInput();

   Vector3d getBodyAngularRateInput();

   Vector3d getPlanarVelocityInput();

}