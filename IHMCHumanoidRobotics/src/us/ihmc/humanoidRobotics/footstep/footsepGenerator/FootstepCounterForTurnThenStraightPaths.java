package us.ihmc.humanoidRobotics.footstep.footsepGenerator;

public class FootstepCounterForTurnThenStraightPaths extends FootstepCounter
{
   static final int numPaths = 2;

   public void initialize(double openingAngle, double closingAngle, double length, double totalAngle, double totalDistance, boolean hipExtensionFirst,
                          boolean isLeftRightMotion, boolean willSetFirstStraightStepToFarSideIfNoTurns, double initialFootDeltaX, double initialFootDeltaY, double initialHipYawOpen, double standardFootWidth)
   {
      reInitializeFields(openingAngle, closingAngle, length, hipExtensionFirst, isLeftRightMotion);
      double overshootStepFraction = generateTurnSteps(totalAngle, initialHipYawOpen, 0);
      boolean straightStepsGenerated = generateStraightSegmentSteps(totalDistance, overshootStepFraction, initialFootDeltaX, initialFootDeltaY, 1, willSetFirstStraightStepToFarSideIfNoTurns, standardFootWidth);
      generateFinalSquareUpSteps(straightStepsGenerated);
   }

   protected int getNumberOfSubPaths()
   {
      return numPaths;
   }

}
