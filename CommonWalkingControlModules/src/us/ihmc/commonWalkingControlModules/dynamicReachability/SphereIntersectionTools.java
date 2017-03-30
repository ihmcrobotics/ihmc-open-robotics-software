package us.ihmc.commonWalkingControlModules.dynamicReachability;

public class SphereIntersectionTools
{
   /**
    * Computes the distance to the intersecting plane between two spheres. Sphere 1 is assumed to be the base sphere, and sphere 2 has it centered at a point
    * separated from the base sphere by {@param sphereSeparation}.
    *
    * @param sphereSeparation distance between the center of the spheres
    * @param radius1 radius of sphere 1
    * @param radius2 radius of sphere 2
    * @return distance from the center of sphere 1 to the intersecting plane between the two spheres
    */
   public static double computeDistanceToIntersectingPlane(double sphereSeparation, double radius1, double radius2)
   {
      return (Math.pow(sphereSeparation, 2.0) - Math.pow(radius2, 2.0) + Math.pow(radius1, 2.0)) / (2.0 * sphereSeparation);
   }

   /**
    * Computes the radius of the intersecting plane between two spheres. Sphere 1 is assumed to be the base sphere, and sphere 2 has it centered at a point
    * separated from the base sphere by {@param sphereSeparation}.
    *
    * @param sphereSeparation distance between the center of the spheres
    * @param radius1 radius of sphere 1
    * @param radius2 radius of sphere 2
    * @return radius of intersecting circular plane between the two spheres.
    */
   public static double computeRadiusOfIntersectingPlane(double sphereSeparation, double radius1, double radius2)
   {
      double distanceToPlane = computeDistanceToIntersectingPlane(sphereSeparation, radius1, radius2);
      return computeEllipseMajorAxisRadius(radius1, distanceToPlane);
   }

   /**
    * The intersection between two spheres is a plane. When this plane is projected onto the ground, it forms an ellipse.
    * This function returns the distance to the center of this ellipse along the vector connect the spheres' centers.
    * Sphere 1 is assumed to be the base sphere, centered at (0, 0). The distance to the center point of sphere 2 is defined
    * by {@param sphereSeparationX} and {@param sphereSeparationZ}.
    *
    * @param sphereSeparationX distance in X to the center of sphere 2 from sphere 1.
    * @param sphereSeparationZ distance in Z to the center of sphere 2 from sphere 1.
    * @param radius1 radius of sphere 1
    * @param radius2 radius of sphere 2
    *
    * @return horizontal distance to the ellipse that is the intersection region projected on the ground.
    */
   public static double computeDistanceToCenterOfIntersectionEllipse(double sphereSeparationX, double sphereSeparationZ, double radius1, double radius2)
   {
      double sphereSeparation = Math.sqrt(Math.pow(sphereSeparationX, 2.0) + Math.pow(sphereSeparationZ, 2.0));
      double angleOfSeparation = Math.atan(sphereSeparationZ / sphereSeparationX);

      return computeDistanceToCenterOfIntersectionEllipseInternal(sphereSeparation, angleOfSeparation, radius1, radius2);
   }

   /**
    * The intersection between two spheres is a plane. When this plane is projected onto the ground, it forms an ellipse.
    * This function returns the distance to the closest edge of this ellipse along the vector connect the spheres' centers.
    * Sphere 1 is assumed to be the base sphere, centered at (0, 0). The distance to the center point of sphere 2 is defined
    * by {@param sphereSeparationX} and {@param sphereSeparationZ}.
    *
    * @param sphereSeparationX distance in X to the center of sphere 2 from sphere 1.
    * @param sphereSeparationZ distance in Z to the center of sphere 2 from sphere 1.
    * @param radius1 radius of sphere 1
    * @param radius2 radius of sphere 2
    *
    * @return horizontal distance to the closest edge of the ellipse that is teh intersection region projected on the ground.
    */
   public static double computeDistanceToNearEdgeOfIntersectionEllipse(double sphereSeparationX, double sphereSeparationZ, double radius1, double radius2)
   {
      if (sphereSeparationZ == 0.0)
         return computeDistanceToIntersectingPlane(sphereSeparationX, radius1, radius2);

      double sphereSeparation = Math.sqrt(Math.pow(sphereSeparationX, 2.0) + Math.pow(sphereSeparationZ, 2.0));
      double angleOfSeparation = Math.atan(sphereSeparationZ / sphereSeparationX);

      double distanceToPlane = computeDistanceToIntersectingPlane(sphereSeparation, radius1, radius2);
      double horizontalDistanceToEllipse = computeDistanceToCenterOfIntersectionEllipseInternal(sphereSeparation, angleOfSeparation, radius1, radius2);

      double minorAxisRadius = computeEllipseMinorAxisRadius(radius1, distanceToPlane, angleOfSeparation);

      return horizontalDistanceToEllipse - minorAxisRadius;
   }

   /**
    * The intersection between two spheres is a plane. When this plane is projected onto the ground, it forms an ellipse.
    * This function returns the distance to farthest edge of this ellipse along the vector connect the spheres' centers.
    * Sphere 1 is assumed to be the base sphere, centered at (0, 0). The distance to the center point of sphere 2 is defined
    * by {@param sphereSeparationX} and {@param sphereSeparationZ}.
    *
    * @param sphereSeparationX distance in X to the center of sphere 2 from sphere 1.
    * @param sphereSeparationZ distance in Z to the center of sphere 2 from sphere 1.
    * @param radius1 radius of sphere 1
    * @param radius2 radius of sphere 2
    *
    * @return horizontal distance to the furthest edge of the ellipse that is teh intersection region projected on the ground.
    */
   public static double computeDistanceToFarEdgeOfIntersectionEllipse(double sphereSeparationX, double sphereSeparationZ, double radius1, double radius2)
   {
      if (sphereSeparationZ == 0.0)
         return computeDistanceToIntersectingPlane(sphereSeparationX, radius1, radius2);

      double sphereSeparation = Math.sqrt(Math.pow(sphereSeparationX, 2.0) + Math.pow(sphereSeparationZ, 2.0));
      double angleOfSeparation = Math.atan(sphereSeparationZ / sphereSeparationX);

      double distanceToPlane = computeDistanceToIntersectingPlane(sphereSeparation, radius1, radius2);
      double horizontalDistanceToEllipse = distanceToPlane * Math.cos(angleOfSeparation);

      double minorAxisRadius = computeEllipseMinorAxisRadius(radius1, distanceToPlane, angleOfSeparation);

      return horizontalDistanceToEllipse + minorAxisRadius;
   }

   static double computeDistanceToCenterOfIntersectionEllipseInternal(double sphereSeparation, double angleOfSeparation, double radius1, double radius2)
   {
      double distanceToPlane = computeDistanceToIntersectingPlane(sphereSeparation, radius1, radius2);
      return distanceToPlane * Math.cos(angleOfSeparation);
   }

   static double computeEllipseMajorAxisRadius(double sphereRadius1, double distanceToPlaneIntersection)
   {
      return Math.sqrt(Math.pow(sphereRadius1, 2.0) - Math.pow(distanceToPlaneIntersection, 2.0));
   }

   static double computeEllipseMinorAxisRadius(double sphereRadius1, double distanceToPlaneIntersection, double angleOfDistanceToHorizontal)
   {
      double majorAxisRadius = computeEllipseMajorAxisRadius(sphereRadius1, distanceToPlaneIntersection);

      return computeEllipseMinorAxisRadius(majorAxisRadius, angleOfDistanceToHorizontal);
   }

   static double computeEllipseMinorAxisRadius(double majorAxisRadius, double angleOfDistanceToHorizontal)
   {
      return majorAxisRadius * Math.sin(angleOfDistanceToHorizontal);
   }
}
