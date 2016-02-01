package us.ihmc.robotics.geometry;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

public class RotationVectorToAxisAngleConverter
{
   private static final double DEFAULT_EPSILON = 2e-12;
   private final Vector3d tempAxisOfRotation = new Vector3d();

   public void convertRotationVectorToAxisAngle(Vector3d rotationVector, AxisAngle4d axisAngleToPack)
   {
      convertRotationVectorToAxisAngle(rotationVector, DEFAULT_EPSILON, axisAngleToPack);
   }

   public void convertRotationVectorToAxisAngle(Vector3d rotationVector, double epsilonNoRotation, AxisAngle4d axisAngleToPack)
   {
      tempAxisOfRotation.set(rotationVector);
      double angularExcursion = tempAxisOfRotation.length();
      if (angularExcursion > epsilonNoRotation)
         tempAxisOfRotation.scale(1.0 / angularExcursion);
      else
      {
         tempAxisOfRotation.set(1.0, 0.0, 0.0);
         angularExcursion = 0.0;
      }

      axisAngleToPack.set(tempAxisOfRotation, angularExcursion);
   }
}
