package us.ihmc.robotics.geometry;

import us.ihmc.commons.AngleTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

/**
 * This class provides methods to generate points evenly distributed on a sphere.
 * Based on the same algorithm, orientations can be generated with the property of exploring evenly the spherical domain.
 * This algorithm can be found in the paper: "Distributing Many Points on a Sphere" by E.B. Saff and B.J. Kuijlaars. (PDF version was on Google on the 02/13/2015).
 * 
 * @author Sylvain
 *
 */
public class SpiralBasedAlgorithm
{

   public static Point3D[] generatePointsOnSphere(double sphereRadius, int numberOfPointsToGenerate)
   {
      return generatePointsOnSphere(new Point3D(), sphereRadius, numberOfPointsToGenerate, computeMagicDeltaN(numberOfPointsToGenerate));
   }

   public static Point3D[] generatePointsOnSphere(double sphereRadius, int numberOfPointsToGenerate, double deltaN)
   {
      return generatePointsOnSphere(new Point3D(), sphereRadius, numberOfPointsToGenerate, deltaN);
   }


   public static Point3D[] generatePointsOnSphere(Point3DReadOnly sphereOrigin, double sphereRadius, int numberOfPointsToGenerate)
   {
      return generatePointsOnSphere(sphereOrigin, sphereRadius, numberOfPointsToGenerate, computeMagicDeltaN(numberOfPointsToGenerate));
   }

   /**
    * Generates a number of points uniformly distributed over the surface of a sphere using a spiral-based approach.
    * This algorithm can be found in the paper: "Distributing Many Points on a Sphere" by E.B. Saff and B.J. Kuijlaars. (PDF version was on Google on the 02/13/2015).
    */
   public static Point3D[] generatePointsOnSphere(Point3DReadOnly sphereOrigin, double sphereRadius, int numberOfPointsToGenerate, double deltaN)
   {
      Point3D[] pointsOnSphere = new Point3D[numberOfPointsToGenerate];

      double phi;
      double previousPhi = 0.0;

      pointsOnSphere[0] = new Point3D();
      pointsOnSphere[0].set(0.0, 0.0, -sphereRadius);
      pointsOnSphere[0].add(sphereOrigin);

      for (int planeIndex = 1; planeIndex < numberOfPointsToGenerate - 1; planeIndex++)
      {
         double unitHeight = -1.0 + 2.0 * planeIndex / (numberOfPointsToGenerate - 1.0);
         double theta = Math.acos(unitHeight);
         phi = previousPhi + deltaN / Math.sqrt(1 - unitHeight * unitHeight);
         AngleTools.trimAngleMinusPiToPi(phi);

         double rSinTheta = sphereRadius * Math.sin(theta);
         pointsOnSphere[planeIndex] = new Point3D();
         pointsOnSphere[planeIndex].setX(rSinTheta * Math.cos(phi));
         pointsOnSphere[planeIndex].setY(rSinTheta * Math.sin(phi));
         pointsOnSphere[planeIndex].setZ(sphereRadius * Math.cos(theta));
         pointsOnSphere[planeIndex].add(sphereOrigin);

         previousPhi = phi;
      }

      pointsOnSphere[numberOfPointsToGenerate - 1] = new Point3D();
      pointsOnSphere[numberOfPointsToGenerate - 1].set(0.0, 0.0, sphereRadius);
      pointsOnSphere[numberOfPointsToGenerate - 1].add(sphereOrigin);

      return pointsOnSphere;
   }

   public static Quaternion[][] generateOrientations(int numberOfRays, int numberOfRotationsAroundRay)
   {
      return generateOrientations(numberOfRays, numberOfRotationsAroundRay, computeMagicDeltaN(numberOfRays));
   }

   public static Quaternion[][] generateOrientations(int numberOfRays, int numberOfRotationsAroundRay, double deltaN)
   {
      Quaternion[][] rotations = new Quaternion[numberOfRays][numberOfRotationsAroundRay];

      Point3D sphereOrigin = new Point3D();
      Vector3D rayThroughSphere = new Vector3D();
      Point3D[] pointsOnSphere = generatePointsOnSphere(0.01, numberOfRays, deltaN);

      Quaternion rotationForXAxisAlignedWithRay = new Quaternion();

      double stepSizeAngleArounRay = 2.0 * Math.PI / numberOfRotationsAroundRay;

      for (int rayIndex = 0; rayIndex < numberOfRays; rayIndex++)
      {
         // Ray that goes from the surface of the sphere to its origin
         rayThroughSphere.sub(sphereOrigin, pointsOnSphere[rayIndex]);
         rayThroughSphere.normalize();

         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.X, rayThroughSphere, rotationForXAxisAlignedWithRay);

         for (int rotationAroundRayIndex = 0; rotationAroundRayIndex < numberOfRotationsAroundRay; rotationAroundRayIndex++)
         {
            double angle = rotationAroundRayIndex * stepSizeAngleArounRay;
            Quaternion rotation = new Quaternion(rotationForXAxisAlignedWithRay);
            rotation.appendRollRotation(angle);
            rotations[rayIndex][rotationAroundRayIndex] = rotation;
         }
      }

      return rotations;
   }

   public static double computeMagicDeltaN(int numberOfPointsToGenerate)
   {
      // Two solutions are suggested for deltaN:
      //      double deltaN = Math.sqrt(8.0 * Math.PI / Math.sqrt(3)) / Math.sqrt(numberOfPoints);
      // From the paper it is said that the following can also be used. It seems to be a magic number at the end.
      double deltaN = 3.6 / Math.sqrt(numberOfPointsToGenerate);
      return deltaN;
   }
}
