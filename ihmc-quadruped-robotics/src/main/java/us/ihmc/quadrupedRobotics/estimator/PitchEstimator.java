package us.ihmc.quadrupedRobotics.estimator;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PitchEstimator
{
   public static double computeGroundPitchFromContacts(QuadrantDependentList<? extends Point3DReadOnly> contacts)
   {
      /* Fit plane to points */
      // Given plane equation Ax+By+z +C = 0
      // find coefficients of plane that best fits the points using least squared in the z direction
      // coefficients returned (A,B,C)
      //pack the plane, and return the squared error
      double n = 0;
      double x = 0;
      double y = 0;
      double z = 0;
      double xx = 0;
      double xy = 0;
      double yy = 0;
      double xz = 0;
      double yz = 0;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         Point3DReadOnly point3d = contacts.get(robotQuadrant);
         n++;
         x += point3d.getX();
         y += point3d.getY();
         z += point3d.getZ();
         xx += Math.pow(point3d.getX(), 2);
         xy += point3d.getX() * point3d.getY();
         xz += point3d.getX() * point3d.getZ();
         yy += Math.pow(point3d.getY(), 2);
         yz += point3d.getY() * point3d.getZ();
      }

      double det = xx * yy * n + 2.0 * xy * y * x - yy * x * x - xy * xy * n - xx * y * y;

      double xx_inv = (n * yy - y * y) / det;
      double xy_inv = -(xy * n - x * y) / det;
      double x_inv = (xy * y - x * yy) / det;
      double yy_inv = (xx * n - x * x) / det;
      double y_inv = -(xx * y - xy * x) / det;
//      double n_inv = (xx - yy - xy * xy) / det;

      double A = -xx_inv * xz - xy_inv * yz - x_inv * z;
      double B = -xy_inv * xz - yy_inv * yz - y_inv * z;
//      double C = -x_inv * xz - y_inv * yz - n_inv * z;

      double nominalYaw = computeNominalYaw(contacts);
      return Math.atan2(Math.cos(nominalYaw) * A + Math.sin(nominalYaw) * B, 1.0);
   }

   public static double computeNominalYaw(QuadrantDependentList<? extends Point3DReadOnly> contactPositions)
   {
      Point3DReadOnly frontLeft = contactPositions.get(RobotQuadrant.FRONT_LEFT);
      Point3DReadOnly frontRight = contactPositions.get(RobotQuadrant.FRONT_RIGHT);
      Point3DReadOnly hindLeft = contactPositions.get(RobotQuadrant.HIND_LEFT);
      Point3DReadOnly hindRight = contactPositions.get(RobotQuadrant.HIND_RIGHT);
      double deltaX = frontLeft.getX() - hindLeft.getX();
      double deltaY = frontLeft.getY() - hindLeft.getY();

      deltaX += frontRight.getX() - hindRight.getX();
      deltaY += frontRight.getY() - hindRight.getY();

      return Math.atan2(deltaY, deltaX);
   }
}
