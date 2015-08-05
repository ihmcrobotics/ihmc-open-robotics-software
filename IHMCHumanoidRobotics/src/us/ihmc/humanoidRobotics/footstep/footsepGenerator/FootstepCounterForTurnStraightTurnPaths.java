package us.ihmc.humanoidRobotics.footstep.footsepGenerator;

public class FootstepCounterForTurnStraightTurnPaths extends FootstepCounter
{
   private static final int numPaths = 3;
 
   public void initialize(double openingAngle, double closingAngle, double stepLength, double totalAngleInitial, double totalDistance, double totalAngleFinal,
                          boolean hipExtensionFirst, boolean isLeftRightMotion, boolean willSetFirstStraightStepToFarSideIfNoTurns, double initialFootDeltaX, double initialFootDeltaY, double initialHipYawOpen, double standardFootWidth)
   {
      reInitializeFields(openingAngle, closingAngle, stepLength, hipExtensionFirst, isLeftRightMotion);
      double overshootStepFraction = generateTurnSteps(totalAngleInitial, initialHipYawOpen, 0);
      boolean straightStepsGenerated = generateStraightSegmentSteps(totalDistance, overshootStepFraction, initialFootDeltaX, initialFootDeltaY, 1, willSetFirstStraightStepToFarSideIfNoTurns, standardFootWidth);
      boolean finalTurnStepsTaken = generateFinalTurningSteps(totalAngleFinal,2);
      generateFinalSquareUpSteps(straightStepsGenerated && !finalTurnStepsTaken);
   }

   private boolean generateFinalTurningSteps(double totalAngle, int currentSubPathIndex)
   {
      int previousStepSize = stepSValues.size();
      generateTurnSteps(totalAngle, 0.0, currentSubPathIndex);

      boolean finalTurnStepsTaken = (previousStepSize < stepSValues.size());

      return finalTurnStepsTaken;
   }

   protected int getNumberOfSubPaths()
   {
      return numPaths;
   }
}
