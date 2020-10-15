package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

public class TriangleTools
{
   public static double computeSideLength(double sideALength, double sideBLength, double interiorAngle)
   {
      double lengthSquared = Math.pow(sideALength, 2.0) + Math.pow(sideBLength, 2.0) - 2.0 * sideALength * sideBLength * Math.cos(interiorAngle);
      return Math.sqrt(lengthSquared);
   }

   public static double computeSideLengthVelocity(double sideALength, double sideBLength, double interiorAngle, double interiorAngleVelocity)
   {
      double sideLength = computeSideLength(sideALength, sideBLength, interiorAngle);
      return computeSideLengthVelocity(sideALength, sideBLength, sideLength, interiorAngle, interiorAngleVelocity);
   }

   public static double computeSideLengthVelocity(double sideALength, double sideBLength, double farSideLength, double interiorAngle,
                                                  double interiorAngleVelocity)
   {
      return sideALength * sideBLength / farSideLength * interiorAngleVelocity * Math.sin(interiorAngle);
   }

   public static double computeSideLengthAcceleration(double sideALength, double sideBLength, double interiorAngle, double interiorAngleVelocity,
                                                      double interiorAngleAcceleration)
   {
      double sideLength = computeSideLength(sideALength, sideBLength, interiorAngle);
      double sideLengthVelocity = computeSideLengthVelocity(sideALength, sideBLength, sideLength, interiorAngle, interiorAngleVelocity);

      return computeSideLengthAcceleration(sideALength, sideBLength, sideLength, sideLengthVelocity, interiorAngle, interiorAngleVelocity,
                                           interiorAngleAcceleration);
   }

   public static double computeSideLengthAcceleration(double sideALength, double sideBLength, double farSideLength, double farSideLengthVelocity,
                                                      double interiorAngle, double interiorAngleVelocity, double interiorAngleAcceleration)
   {
      double sideLengthAcceleration = interiorAngleAcceleration * Math.sin(interiorAngle) + Math.pow(interiorAngleVelocity, 2.0) * Math.cos(interiorAngle);
      sideLengthAcceleration *= sideALength * sideBLength / farSideLength;
      sideLengthAcceleration -= Math.pow(farSideLengthVelocity, 2.0) / farSideLength;

      return sideLengthAcceleration;
   }

   public static double computeInteriorAngle(double sideALength, double sideBLength, double farSideLength)
   {
      double delta = Math.pow(farSideLength, 2.0) - Math.pow(sideALength, 2.0) - Math.pow(sideBLength, 2.0);
      return Math.acos(-delta / (2.0 * sideALength * sideBLength));
   }

   public static double computeInteriorAngleVelocity(double sideALength, double sideBLength, double farSideLength, double farSideVelocity)
   {
      double interiorAngle = computeInteriorAngle(sideALength, sideBLength, farSideLength);
      return computeInteriorAngleVelocity(sideALength, sideBLength, farSideLength, farSideVelocity, interiorAngle);
   }

   public static double computeInteriorAngleVelocity(double sideALength, double sideBLength, double farSideLength, double farSideVelocity, double interiorAngle)
   {
      return farSideLength * farSideVelocity / (sideALength * sideBLength * Math.sin(interiorAngle));
   }

   public static double computeInteriorAngleAcceleration(double sideALength, double sideBLength, double farSideLength, double farSideVelocity,
                                                         double farSideAcceleration)
   {
      double interiorAngle = computeInteriorAngle(sideALength, sideBLength, farSideLength);
      double interiorAngleVelocity = computeInteriorAngleVelocity(sideALength, sideBLength, farSideLength, farSideVelocity, interiorAngle);
      return computeInteriorAngleAcceleration(sideALength, sideBLength, farSideLength, farSideVelocity, farSideAcceleration, interiorAngle,
                                              interiorAngleVelocity);
   }

   public static double computeInteriorAngleAcceleration(double sideALength, double sideBLength, double farSideLength, double farSideVelocity,
                                                         double farSideAcceleration, double interiorAngle, double interiorAngleVelocity)
   {
      double interiorAngleAcceleration =
            Math.pow(farSideVelocity, 2.0) + farSideLength * farSideAcceleration;
      interiorAngleAcceleration /= sideALength * sideBLength * Math.sin(interiorAngle);
      interiorAngleAcceleration -= Math.pow(interiorAngleVelocity, 2.0) / Math.tan(interiorAngle);
      return interiorAngleAcceleration;
   }

   /**
    * Calculate an unknown side length of a fully defined 2D Triangle by the law of Sine.
    * <p>
    * Given a triangle with the three sides a, b, and c, this methods calculates the length of the side
    * c, given:
    * <ul>
    * <li>the lengths of a and b.
    * <li>the angle opposite of side b, angle B.
    * </ul>
    * </p>
    *
    * @param sideALength the length of the side a.
    * @param sideBLength the length of the side b.
    * @param angleB the angle opposite side b.
    * @param isAngleAObtuse Whether the angle opposite of side a is obtuse or not, only used if there are two possible triangles
    * @return the value of the unknown side length.
    */
   public static double computeSideLengthFromSideSideAngle(double sideALength, double sideBLength, double angleB, boolean isAngleAObtuse)
   {
      double lawSinesB;
      double angleA;
      double angleC;
      if (angleB >= Math.PI/2)
      {
         if (sideALength >= sideBLength)
         {
            return Double.NaN;
         }
      }
      else
      {
         if (sideALength > sideBLength)
         {
            double height = sideALength * Math.sin(angleB);
            if (sideBLength < height)
            {
               return Double.NaN;
            }
            else if (sideBLength > height)
            {
               lawSinesB = Math.sin(angleB) / sideBLength;
               if (isAngleAObtuse)
               {
                  angleA = Math.PI - Math.asin(lawSinesB * sideALength);
               }
               else
               {
                  angleA = Math.asin(lawSinesB * sideALength);
               }
               angleC = Math.PI - angleA - angleB;
               return Math.sin(angleC) / lawSinesB;
            }
         }
      }
      lawSinesB = Math.sin(angleB) / sideBLength;
      angleA = Math.asin(lawSinesB * sideALength);
      angleC = Math.PI - angleA - angleB;
      return Math.sin(angleC) / lawSinesB;
   }
}
