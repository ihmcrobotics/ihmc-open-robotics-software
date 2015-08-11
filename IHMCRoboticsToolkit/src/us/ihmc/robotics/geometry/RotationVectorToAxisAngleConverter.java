package us.ihmc.robotics.geometry;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

public class RotationVectorToAxisAngleConverter
{
   private static final double DEFAULT_EPSILON = 2e-12;
   private final Vector3d tempAxisOfRotation = new Vector3d();

   public void convertRotationVectorToAxisAngle(AxisAngle4d axisAngleToPack, Vector3d rotationVector)
   {
      convertRotationVectorToAxisAngle(axisAngleToPack, rotationVector, DEFAULT_EPSILON);
   }

   public void convertRotationVectorToAxisAngle(AxisAngle4d axisAngleToPack, Vector3d rotationVector, double epsilonNoRotation)
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
