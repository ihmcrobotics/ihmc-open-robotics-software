package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

public interface TranslationalPathParameters {
   // The nominal lateral distance between feet.
   // For forward walking this is the lateral distance between feet.
   // For side stepping this is the minimum distance between feet.
   public double getNominalStepWidth();
   public double getMinimumStepWidth();

   // Used to find periodic displacement along path direction
   // This is the step displacement.
   // For side stepping the largest distance between feet will be the minStepWidth + sideStepLength
   public double getForwardStepLength();
   public double getBackwardStepLength();
   public double getSidewardStepLength();
}
