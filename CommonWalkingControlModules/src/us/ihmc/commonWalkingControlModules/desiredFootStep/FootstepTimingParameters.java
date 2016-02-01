package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;

public class FootstepTimingParameters
{
   private double footstepPathSwingTime;
   private double slowBlindWalkingSwingTime;
   private double blindWalkingInMudSwingTime;

   private double footstepPathTransferTime;
   private double slowBlindWalkingTransferTime;
   private double blindWalkingInMudTransferTime;
   
   private FootstepTimingParameters()
   {

   }

   public static FootstepTimingParameters createForSlowWalkingOnRobot(WalkingControllerParameters walkingControllerParameters)
   {
      FootstepTimingParameters footstepTimingParameters = new FootstepTimingParameters();

      footstepTimingParameters.setFootstepPathSwingTime(walkingControllerParameters.getDefaultSwingTime());
      footstepTimingParameters.setSlowBlindWalkingSwingTime(walkingControllerParameters.getDefaultSwingTime());
      footstepTimingParameters.setBlindWalkingInMudSwingTime(walkingControllerParameters.getDefaultSwingTime());

      footstepTimingParameters.setFootstepPathTransferTime(walkingControllerParameters.getDefaultTransferTime());
      footstepTimingParameters.setSlowBlindWalkingTransferTime(walkingControllerParameters.getDefaultTransferTime());
      footstepTimingParameters.setBlindWalkingInMudTransferTime(walkingControllerParameters.getDefaultTransferTime());

      return footstepTimingParameters;
   }

   public static FootstepTimingParameters createForFastWalkingInSimulation(WalkingControllerParameters walkingControllerParameters)
   {
      FootstepTimingParameters footstepTimingParameters = new FootstepTimingParameters();

      footstepTimingParameters.setFootstepPathSwingTime(walkingControllerParameters.getDefaultSwingTime());
      footstepTimingParameters.setSlowBlindWalkingSwingTime(0.8);
      footstepTimingParameters.setBlindWalkingInMudSwingTime(2.2);

      footstepTimingParameters.setFootstepPathTransferTime(walkingControllerParameters.getDefaultTransferTime());
      footstepTimingParameters.setSlowBlindWalkingTransferTime(0.35);
      footstepTimingParameters.setBlindWalkingInMudTransferTime(0.6);

      return footstepTimingParameters;
   }

   public void setSwingTime(double swingTime) 
   {
      if (swingTime < 0.01) 
      {
         return;
      }
      this.footstepPathSwingTime = swingTime;
   }
   
   public void setTransferTime(double transferTime) 
   {
      if (transferTime < 0.01) 
      {
         return;
      }
      this.footstepPathTransferTime = transferTime;
   }

   public double getFootstepPathSwingTime()
   {
      return footstepPathSwingTime;
   }

   public void setFootstepPathSwingTime(double footstepPathSwingTime)
   {
      this.footstepPathSwingTime = footstepPathSwingTime;
   }

   public double getSlowBlindWalkingSwingTime()
   {
      return slowBlindWalkingSwingTime;
   }

   public void setSlowBlindWalkingSwingTime(double slowBlindWalkingSwingTime)
   {
      this.slowBlindWalkingSwingTime = slowBlindWalkingSwingTime;
   }

   public double getBlindWalkingInMudSwingTime()
   {
      return blindWalkingInMudSwingTime;
   }

   public void setBlindWalkingInMudSwingTime(double blindWalkingInMudSwingTime)
   {
      this.blindWalkingInMudSwingTime = blindWalkingInMudSwingTime;
   }

   public double getFootstepPathTransferTime()
   {
      return footstepPathTransferTime;
   }

   public void setFootstepPathTransferTime(double footstepPathTransferTime)
   {
      this.footstepPathTransferTime = footstepPathTransferTime;
   }

   public double getSlowBlindWalkingTransferTime()
   {
      return slowBlindWalkingTransferTime;
   }

   public void setSlowBlindWalkingTransferTime(double slowBlindWalkingTransferTime)
   {
      this.slowBlindWalkingTransferTime = slowBlindWalkingTransferTime;
   }

   public double getBlindWalkingInMudTransferTime()
   {
      return blindWalkingInMudTransferTime;
   }

   public void setBlindWalkingInMudTransferTime(double blindWalkingInMudTransferTime)
   {
      this.blindWalkingInMudTransferTime = blindWalkingInMudTransferTime;
   }

   @Deprecated
   public static FootstepTimingParameters createForFastWalkingInSimulationForTest()
   {
      FootstepTimingParameters footstepTimingParameters = new FootstepTimingParameters();

      footstepTimingParameters.setFootstepPathSwingTime(0.6);
      footstepTimingParameters.setSlowBlindWalkingSwingTime(0.8);
      footstepTimingParameters.setBlindWalkingInMudSwingTime(2.2);

      footstepTimingParameters.setFootstepPathTransferTime(0.3);
      footstepTimingParameters.setSlowBlindWalkingTransferTime(0.35);
      footstepTimingParameters.setBlindWalkingInMudTransferTime(0.6);

      return footstepTimingParameters;
   }
}
