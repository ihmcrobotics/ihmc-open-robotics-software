package us.ihmc.humanoidRobotics.footstep.footsepGenerator;

public class FootstepCounterForSingleTurnPath extends FootstepCounter
{
   private static final int numPaths = 1;
   private double overshootStepFraction = 1.0;
   private boolean finalTurnStepsTaken = false;
   
   public void initialize(double openingAngle, double closingAngle, double totalAngle, boolean hipExtensionFirst, double initialHipYawOpen)
   {
      reInitializeFields(openingAngle, closingAngle, 0, hipExtensionFirst, false);
      overshootStepFraction = generateTurnSteps(totalAngle, initialHipYawOpen, 0);
      finalTurnStepsTaken = (stepSValues.size()>0);
      generateFinalSquareUpSteps(false);
   }

   protected int getNumberOfSubPaths()
   {
      return numPaths;
   }

   public boolean turnStepsTaken() {
      return finalTurnStepsTaken;
   }

   public double getOvershootStepFraction() {
      return overshootStepFraction;
   }

}
