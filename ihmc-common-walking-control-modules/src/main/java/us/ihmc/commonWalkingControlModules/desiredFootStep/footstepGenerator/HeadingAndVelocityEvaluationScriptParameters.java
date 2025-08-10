package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

public class HeadingAndVelocityEvaluationScriptParameters
{
   private double acceleration = 0.25;
   private double maxVelocity = 1.0;
   private double cruiseVelocity = 0.6;
   private double headingDot = 0.5;
   private double sideStepVelocity = 0.4;
   private double maxHeadingDot = 0.1;
   private double maxTurningVelocity = 10.0 * maxHeadingDot;
   private double turningAcceleration = 1.0;

   public double getAcceleration()
   {
      return acceleration ;
   }

   public void setAcceleration(double acceleration)
   {
      this.acceleration = acceleration;
   }

   public double getMaxVelocity()
   {
      return maxVelocity ;
   }

   public void setMaxVelocity(double maxVelocity)
   {
      this.maxVelocity = maxVelocity;
   }

   public double getCruiseVelocity()
   {
      return cruiseVelocity ;
   }

   public void setCruiseVelocity(double cruiseVelocity)
   {
      this.cruiseVelocity = cruiseVelocity;
   }

   public double getHeadingDot()
   {
      return headingDot ;
   }

   public void setHeadingDot(double headingDot)
   {
      this.headingDot = headingDot;
   }
   
   public double getSideStepVelocity()
   {
      return sideStepVelocity;
   }

   public void setSideStepVelocity(double sideStepVelocity)
   {
      this.sideStepVelocity = sideStepVelocity;
   }
   
   public double getMaxHeadingDot()
   {
      return maxHeadingDot;
   }

   public void setMaxHeadingDot(double maxHeadingDot)
   {
      this.maxHeadingDot = maxHeadingDot;
   }

   public double getMaxTurningVelocity(){ return maxTurningVelocity;}

   public double getTurningAcceleration(){ return turningAcceleration;}

   public void setTurningAcceleration(double turningAcceleration)
   {
      this.turningAcceleration = turningAcceleration;
   }

   public void setMaxTurningVelocity(double maxTurningVelocity){this.maxTurningVelocity = maxTurningVelocity; }


}
