package us.ihmc.commonWalkingControlModules.configurations;

public class PelvisOffsetWhileWalkingParameters
{
   /**
    * Whether or not to utilize pelvis orientation angle modifications when walking to create natural pelvis motion
    */
   public boolean addPelvisOrientationOffsetsFromWalkingMotion()
   {
      return true;
   }

   /**
    * Multiplier of the step angle to determine the desired pelvis yaw magnitude when walking
    */
   public double getPelvisYawRatioOfStepAngle()
   {
      return 0.2;
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
      return 0.2;
   }

   public double getPercentOfSwingPitchingFromUpcomingLeg()
   {
      return 0.2;
   }

   public double getPercentOfSwingPitchingFromSwingLeg()
   {
      return 0.2;
   }
}
