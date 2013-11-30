package us.ihmc.commonWalkingControlModules.desiredFootStep;

public class FootstepTimingParameters
{
   private static final double SWING_RANGE = 2.0;
   private static final double TRANSFER_RANGE = 3.0;

   private double speed = 0.0;
   
   private double footstepPathSwingTime = 0.6;
   private double slowBlindWalkingSwingTime = 0.8;
   private double blindWalkingInMudSwingTime = 2.2;

   private double footstepPathTransferTime = 0.25;
   private double slowBlindWalkingTransferTime = 0.35;
   private double blindWalkingInMudTransferTime = 0.6;
   
   public FootstepTimingParameters()
   {

   }

   public static FootstepTimingParameters createForSlowWalkingOnRobot()
   {
      FootstepTimingParameters footstepTimingParameters = new FootstepTimingParameters();

      footstepTimingParameters.setFootstepPathSwingTime(1.5);
      footstepTimingParameters.setSlowBlindWalkingSwingTime(1.5);
      footstepTimingParameters.setBlindWalkingInMudSwingTime(1.5);

      footstepTimingParameters.setFootstepPathTransferTime(1.5);
      footstepTimingParameters.setSlowBlindWalkingTransferTime(1.5);
      footstepTimingParameters.setBlindWalkingInMudTransferTime(1.5);

      return footstepTimingParameters;
   }

   public static FootstepTimingParameters createForFastWalkingInSimulation()
   {
      FootstepTimingParameters footstepTimingParameters = new FootstepTimingParameters();

      footstepTimingParameters.setFootstepPathSwingTime(0.6);
      footstepTimingParameters.setSlowBlindWalkingSwingTime(0.8);
      footstepTimingParameters.setBlindWalkingInMudSwingTime(2.2);

      footstepTimingParameters.setFootstepPathTransferTime(0.25);
      footstepTimingParameters.setSlowBlindWalkingTransferTime(0.35);
      footstepTimingParameters.setBlindWalkingInMudTransferTime(0.6);

      return footstepTimingParameters;
   }

   // Expects 0 to 1
   public void setSpeed(double speed)
   {
      this.speed = -2.0*(speed-0.5);
   }

   public double getFootstepPathSwingTime()
   {
      return footstepPathSwingTime * Math.pow(SWING_RANGE, speed);
   }

   public void setFootstepPathSwingTime(double footstepPathSwingTime)
   {
      this.footstepPathSwingTime = footstepPathSwingTime;
   }

   public double getSlowBlindWalkingSwingTime()
   {
      return slowBlindWalkingSwingTime * Math.pow(SWING_RANGE, speed);
   }

   public void setSlowBlindWalkingSwingTime(double slowBlindWalkingSwingTime)
   {
      this.slowBlindWalkingSwingTime = slowBlindWalkingSwingTime;
   }

   public double getBlindWalkingInMudSwingTime()
   {
      return blindWalkingInMudSwingTime * Math.pow(SWING_RANGE, speed);
   }

   public void setBlindWalkingInMudSwingTime(double blindWalkingInMudSwingTime)
   {
      this.blindWalkingInMudSwingTime = blindWalkingInMudSwingTime;
   }

   public double getFootstepPathTransferTime()
   {
      return footstepPathTransferTime * Math.pow(TRANSFER_RANGE, speed);
   }

   public void setFootstepPathTransferTime(double footstepPathTransferTime)
   {
      this.footstepPathTransferTime = footstepPathTransferTime;
   }

   public double getSlowBlindWalkingTransferTime()
   {
      return slowBlindWalkingTransferTime * Math.pow(TRANSFER_RANGE, speed);
   }

   public void setSlowBlindWalkingTransferTime(double slowBlindWalkingTransferTime)
   {
      this.slowBlindWalkingTransferTime = slowBlindWalkingTransferTime;
   }

   public double getBlindWalkingInMudTransferTime()
   {
      return blindWalkingInMudTransferTime * Math.pow(TRANSFER_RANGE, speed);
   }

   public void setBlindWalkingInMudTransferTime(double blindWalkingInMudTransferTime)
   {
      this.blindWalkingInMudTransferTime = blindWalkingInMudTransferTime;
   }

}
