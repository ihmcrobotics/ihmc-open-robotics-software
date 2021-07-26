package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

public class UIFootstepGeneratorParameters
{
   //used for straight line footstep path generation in the UI (clicking in the window)
   //defaults work with atlas and val
   protected double straightstepLength = 0.4;
   protected double straightStepWidth = 0.25;
   protected double reverseStepLength = 0.15;
   protected double reverseStepWidth = 0.25;
   protected double shuffleStepLength = 0.25;
   protected double shuffleStepWidth = 0.21;
   protected double turningStepWidth = 0.2;

   public double getStraightstepLength()
   {
      return straightstepLength;
   }

   public double getStraightStepWidth()
   {
      return straightStepWidth;
   }

   public double getReverseStepLength()
   {
      return reverseStepLength;
   }

   public double getReverseStepWidth()
   {
      return reverseStepWidth;
   }

   public double getShuffleStepLength()
   {
      return shuffleStepLength;
   }

   public double getShuffleStepWidth()
   {
      return shuffleStepWidth;
   }

   public double getTurningStepWidth()
   {
      return turningStepWidth;
   }
}
