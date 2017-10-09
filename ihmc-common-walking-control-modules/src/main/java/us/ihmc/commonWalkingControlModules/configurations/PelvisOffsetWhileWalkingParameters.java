package us.ihmc.commonWalkingControlModules.configurations;

public class PelvisOffsetWhileWalkingParameters
{
   /**
    * Whether or not to utilize pelvis orientation angle modifications when walking to create natural pelvis motion
    */
   public boolean addPelvisOrientationOffsetsFromWalkingMotion()
   {
      return false;
   }

   /**
    * Multiplier of the step angle to determine the desired pelvis yaw magnitude when walking
    */
   public double getPelvisYawRatioOfStepAngle()
   {
      return 0.25;
   }

   /**
    * Step length in meters to add the pelvis yawing motion
    */
   public double getStepLengthToAddYawingMotion()
   {
      return 0.03;
   }

   /**
    * Multiplier of the leg angle to determine the desired pelvis pitch magnitude when walking
    */
   public double getPelvisPitchRatioOfLegAngle()
   {
      return 0.3;
   }

   /**
    * Fraction of the end part of the swing phase in which to interpolate the pelvis tilt between the stance leg
    * and the upcoming stance leg. Effectively forces the pelvis to "turn over" sooner.
    * Must be between 0.0 and 1.0.
    */
   public double getFractionOfSwingPitchingFromUpcomingLeg()
   {
      return 0.15;
   }

   /**
    * Fraction of the first part of the swing phase in which to interpolate the pelvis tilt between the stance leg
    * and the swing leg. Effectively delays the pelvis from "turning over".
    * Must be between 0.0 and 1.0.
    */
   public double getFractionOfSwingPitchingFromSwingLeg()
   {
      return 0.2;
   }
}
