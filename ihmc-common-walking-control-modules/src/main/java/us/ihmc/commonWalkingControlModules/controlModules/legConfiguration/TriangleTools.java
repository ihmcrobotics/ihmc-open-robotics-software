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
}
