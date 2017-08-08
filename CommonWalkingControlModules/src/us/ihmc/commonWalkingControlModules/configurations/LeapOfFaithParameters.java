package us.ihmc.commonWalkingControlModules.configurations;

public class LeapOfFaithParameters
{
   /**
    * Specifies whether or not to scale the weights in the optimization on the swing foot near the end of swing.
    */
   public boolean scaleFootWeight()
   {
      return false;
   }

   /**
    * The fraction of the way through the swing phase at which the pelvis starts to rotate down and in the
    * direction of the step
    */
   public double getFractionOfSwingToScaleFootWeight()
   {
      return 0.92;
   }

   /**
    * This is the rate factor that the foot weight is scaled by, via reduction, as a function of the time it's being scaled.
    */
   public double getHorizontalFootWeightScaleFactor()
   {
      return 0.0;
   }

   /**
    * This is the rate factor that the foot weight is scaled by, via increase, as a function of the time it's being scaled.
    */
   public double getVerticalFootWeightScaleFactor()
   {
      return 20.0;
   }

   /**
    * This is the minimum horizontal weight that will be used for the swing foot.
    */
   public double getMinimumHorizontalFootWeight()
   {
      return 2.5;
   }

   /**
    * Specifies whether or not to start rotating the pelvis to set the foot down.
    */
   public boolean usePelvisRotation()
   {
      return false;
   }

   /**
    * Gain to modify the desired pelvis yaw cause the pelvis to yaw towards the step to help set the foot down.
    * It is multiplied by the time past when we expected touchdown to compute the angle offset of the pelvis yaw.
    */
   public double getPelvisReachingYawGain()
   {
      return 1.0;
   }

   /**
    * Gain to modify the desired pelvis roll cause the pelvis to roll towards the step to help set the foot down.
    * It is multiplied by the time past when we expected touchdown to compute the angle offset of the pelvis roll.
    */
   public double getPelvisReachingRollGain()
   {
      return 1.0;
   }

   /**
    * The maximum angle that the pelvis will yaw in the direction of the step.
    */
   public double getPelvisReachingMaxYaw()
   {
      return 0.2;
   }

   /**
    * This maximum angle that the pelvis will roll down.
    */
   public double getPelvisReachingMaxRoll()
   {
      return 0.3;
   }

   /**
    * The fraction of the way through the swing phase at which the pelvis starts to rotate down and in the
    * direction of the step
    */
   public double getPelvisReachingFractionOfSwing()
   {
      return 0.90;
   }

   public boolean relaxPelvisControl()
   {
      return false;
   }

   public double getRelaxationRate()
   {
      return 2.0;
   }

   public double getMinimumPelvisWeight()
   {
      return 1.0;
   }
}
