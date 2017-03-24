package us.ihmc.commonWalkingControlModules.dynamicReachability;

public class SphereIntersectionTools
{
   /**
    * Computes the distance to the intersecting plane between two spheres. Sphere 1 is assumed to be the base sphere, and sphere 2 has it centered at a point
    * separated from the base sphere by {@param fociiSeparation}.
    *
    * @param fociiSeparation distance between the center of the spheres
    * @param radius1 radius of sphere 1
    * @param radius2 radius of sphere 2
    * @return distance from the center of sphere 1 to the intersecting plane between the two spheres
    */
   public static double computeDistanceToIntersectingPlane(double fociiSeparation, double radius1, double radius2)
   {
      return (Math.pow(fociiSeparation, 2.0) - Math.pow(radius2, 2.0) + Math.pow(radius1, 2.0)) / (2.0 * fociiSeparation);
   }

   /**
    * Computes the radius of the intersecting plane between two spheres. Sphere 1 is assumed to be the base sphere, and sphere 2 has it centered at a point
    * separated from the base sphere by {@param fociiSeparation}.
    *
    * @param fociiSeparation distance between the center of the spheres
    * @param radius1 radius of sphere 1
    * @param radius2 radius of sphere 2
    * @return radius of intersecting circular plane between the two spheres.
    */
   public static double computeRadiusOfIntersectingPlane(double fociiSeparation, double radius1, double radius2)
   {
      double distanceToPlane = computeDistanceToIntersectingPlane(fociiSeparation, radius1, radius2);

      return Math.sqrt(Math.pow(radius1, 2.0) - Math.pow(distanceToPlane, 2.0));
   }

   public static double computeMinimumDistanceToIntersectingPlane(double fociiSeparationX, double fociiSeparationZ, double radius1, double radius2)
   {
      double fociiSeparation = Math.sqrt(Math.pow(fociiSeparationX, 2.0) + Math.pow(fociiSeparationZ, 2.0));
      double angleOfSeperation = Math.atan(fociiSeparationZ / fociiSeparationX);

      double distanceToPlane = computeDistanceToIntersectingPlane(fociiSeparation, radius1, radius2);
      double radiusOfPlane = Math.sqrt(Math.pow(radius1, 2.0) - Math.pow(distanceToPlane, 2.0));

      return distanceToPlane * Math.cos(angleOfSeperation) - radiusOfPlane * Math.sin(angleOfSeperation);
   }

   public static double computeMaximumDistanceToIntersectingPlane(double fociiSeparationX, double fociiSeparationZ, double radius1, double radius2)
   {
      double fociiSeparation = Math.sqrt(Math.pow(fociiSeparationX, 2.0) + Math.pow(fociiSeparationZ, 2.0));
      double angleOfSeperation = Math.atan(fociiSeparationZ / fociiSeparationX);

      double distanceToPlane = computeDistanceToIntersectingPlane(fociiSeparation, radius1, radius2);
      double radiusOfPlane = Math.sqrt(Math.pow(radius1, 2.0) - Math.pow(distanceToPlane, 2.0));

      return distanceToPlane * Math.cos(angleOfSeperation) - radiusOfPlane * Math.sin(angleOfSeperation);
   }
}
