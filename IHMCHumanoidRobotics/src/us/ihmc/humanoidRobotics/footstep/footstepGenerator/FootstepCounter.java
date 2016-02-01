package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.MathTools;

public abstract class FootstepCounter
{
   private static final String DEBUG_SIGNATURE = "FootstepCounter\n\t";
   protected double closingAngle;
   private final boolean DEBUG = false;
   protected boolean hipExtensionFirst;
   protected double openingAngle;
   protected ArrayList<Double> stepSValues = new ArrayList<Double>();
   protected ArrayList<Double> linearToTurningRatios = new ArrayList<Double>();
   protected ArrayList<Integer> subPathIndices = new ArrayList<Integer>();
   protected double stepLength;
   protected int indexOfFirstStraightPathStep = -1;
   protected boolean isLeftRightMotion;

   public List<Double> getSValues2()
   {
      return stepSValues;
   }

   public List<Double> getlinearToTurningRatios()
   {
      return linearToTurningRatios;
   }

   public List<Integer> getSubPathIndices() {
      return subPathIndices;
   }

   public int getFirstStraightPathStepIndex()
   {
      return indexOfFirstStraightPathStep;
   }

   protected void reInitializeFields(double openingAngle, double closingAngle, double length, boolean hipExtensionFirst, boolean isLeftRightMotion)
   {
      stepSValues.clear();
      linearToTurningRatios.clear();
      subPathIndices.clear();
      this.openingAngle = openingAngle;
      this.closingAngle = closingAngle;
      this.stepLength = length;
      this.hipExtensionFirst = hipExtensionFirst;
      this.isLeftRightMotion = isLeftRightMotion;
      indexOfFirstStraightPathStep = -1;
   }

   protected double generateTurnSteps(double totalAngle, double initialHipYawOpen, int currentSubPathIndex)
   {
      double overshootStepFraction;
      if (MathTools.epsilonEquals(0.0, totalAngle, 1e-8))
      {
         overshootStepFraction = 1.0;

         return overshootStepFraction;
      }

      double openingDeltaS = openingAngle / Math.abs(totalAngle);
      double closingDeltaS = closingAngle / Math.abs(totalAngle);
      double initialOffsetDeltaS = Math.abs((initialHipYawOpen / 2) / totalAngle);
      
      boolean hipExtensionSecond = !hipExtensionFirst;
      double firstTurningStepDeltaS = hipExtensionFirst ? openingDeltaS : closingDeltaS;
      double secondTurningStepDeltaS = hipExtensionSecond ? openingDeltaS : closingDeltaS;

      double deltaSForPairOfTurningSteps = closingDeltaS + openingDeltaS;
      double totalS = 1 + initialOffsetDeltaS;
      double pairsOfTurningSteps = totalS / deltaSForPairOfTurningSteps;
      int minimumPairsOfTurningSteps = (int) Math.floor(pairsOfTurningSteps);
      double sRemaining = totalS - (minimumPairsOfTurningSteps) * deltaSForPairOfTurningSteps;

      printIfDebug("pairsOfTurningSteps =" + pairsOfTurningSteps);

      int totalPureTurningSteps;
      if (firstTurningStepDeltaS > sRemaining)
      {
         totalPureTurningSteps = 2 * minimumPairsOfTurningSteps;
         overshootStepFraction = 1 - sRemaining / firstTurningStepDeltaS;
      }
      else
      {
         totalPureTurningSteps = 2 * minimumPairsOfTurningSteps + 1;
         sRemaining = (sRemaining - firstTurningStepDeltaS);
         overshootStepFraction = 1 - sRemaining / secondTurningStepDeltaS;
      }

      printIfDebug("sRemaining = " + sRemaining + "\n\t" + "overshoot = " + overshootStepFraction);


      double currentS = -initialOffsetDeltaS;
      if (initialOffsetDeltaS > firstTurningStepDeltaS)
         printIfDebug("Initial foot yaw offset is greater than the first turning step yaw and will result in a negative path S");

      for (int i = 0; i < totalPureTurningSteps; i++)
      {
         currentS += isEvenStep(i) ? firstTurningStepDeltaS : secondTurningStepDeltaS;
         addSubpathStep(currentS, 0.0, currentSubPathIndex);
      }

      return overshootStepFraction;
   }

   protected boolean generateStraightSegmentSteps(double totalDistance, double overshootStepFraction, double initialFootDeltaX, double initialFootDeltaY,
           int currentSubPathIndex, boolean willSetFirstStraightStepToFarSideIfNoTurns, double standardFootWidth)
   {
      if (MathTools.epsilonEquals(0.0, totalDistance, 1e-7))
      {
         printIfDebug("small straight segment, skipping straight steps");
         return false;
      }

      if (isLeftRightMotion && (stepSValues.size() > 0))
      {
         addSubpathStep(0, 1.0, currentSubPathIndex);
         addSubpathStep(0, 1.0, currentSubPathIndex);    
         // This last step may be deleted higher up, if it is the same side as the next step... Just need to know direction of travel...
         // indexOfFirstStraightPathStep will help identify this step

         overshootStepFraction = 1.0;
      }

      indexOfFirstStraightPathStep = stepSValues.size();

      double initialFeetOffsetAdjustment = 0;
      boolean leftRightMotionFarFootStep = false;
      if (indexOfFirstStraightPathStep == 0)
      {
         if (isLeftRightMotion)
         {
            if (willSetFirstStraightStepToFarSideIfNoTurns)
            {
               initialFeetOffsetAdjustment = (initialFootDeltaY - standardFootWidth) / 2.0;    // far foot adjustment (will require extra step or one less...)
               leftRightMotionFarFootStep = true;
            }
            else
            {
               initialFeetOffsetAdjustment = (standardFootWidth - initialFootDeltaY) / 2.0;    // near foot adjustment (correct)
               leftRightMotionFarFootStep = false;
            }
         }
         else
         {
            if (willSetFirstStraightStepToFarSideIfNoTurns)
               initialFeetOffsetAdjustment = Math.abs(initialFootDeltaX) / 2.0;    // far foot adjustment
            else
               initialFeetOffsetAdjustment = -Math.abs(initialFootDeltaX) / 2.0;    // near foot adjustment (potential for negative pathS)
         }
      }

      double estimatedNumberOfStraightSteps = Math.max(0, (totalDistance - initialFeetOffsetAdjustment)) / stepLength;
      int straightStepsBesidesOvershoot = (int) Math.max(0, Math.ceil(estimatedNumberOfStraightSteps - overshootStepFraction));
      double actualStepLength = (totalDistance - initialFeetOffsetAdjustment) / (straightStepsBesidesOvershoot + overshootStepFraction);
      double actualDeltaSPerStep = actualStepLength / totalDistance;
      double currentStraightSValue = initialFeetOffsetAdjustment / totalDistance + overshootStepFraction * actualDeltaSPerStep;

      if (leftRightMotionFarFootStep)
      {
         currentStraightSValue-=actualDeltaSPerStep;
         straightStepsBesidesOvershoot ++;
      }

      addSubpathStep(currentStraightSValue, overshootStepFraction, currentSubPathIndex);
      if (isLeftRightMotion && !leftRightMotionFarFootStep)
         addSubpathStep(currentStraightSValue, 1.0, currentSubPathIndex);

      for (int i = 0; i < straightStepsBesidesOvershoot; i++)
      {
         currentStraightSValue += actualDeltaSPerStep;
         addSubpathStep(currentStraightSValue, 1.0, currentSubPathIndex);
         if (isLeftRightMotion)
            addSubpathStep(currentStraightSValue, 1.0, currentSubPathIndex);
      }

      printIfDebug("currentStraightSValue after finishing straight steps = " + currentStraightSValue);
      
      return true;
   }

   protected void addSubpathStep(double subPathS, double linearToTurningRatio, int pathSegmentIndex)
   {
      stepSValues.add(subPathS);
      linearToTurningRatios.add(linearToTurningRatio);
      subPathIndices.add(pathSegmentIndex);
   }

   protected void removeSubpathStep(int indexToRemove)
   {
      stepSValues.remove(indexToRemove);
      linearToTurningRatios.remove(indexToRemove);
      subPathIndices.remove(indexToRemove);
   }

   protected abstract int getNumberOfSubPaths();

   protected boolean isEvenStep(int i)
   {
      return i % 2 == 0;
   }

   protected void printIfDebug(String message)
   {
      if (DEBUG)
      {
         System.out.println(DEBUG_SIGNATURE + message);
      }
   }

   ArrayList<Double> getStepSValues()
   {
      return new ArrayList<Double>(this.stepSValues);
   }

   void setStepSValues(ArrayList<Double> stepSValues)
   {
      this.stepSValues.clear();
      this.stepSValues.addAll(stepSValues);
   }

   protected void generateFinalSquareUpSteps(boolean lastStepsAreStraightSteps) {
      removeFinalStraightStepsToBeSquared(lastStepsAreStraightSteps);
   
      generateFinalSquareUpStep();
      generateFinalSquareUpStep();
   }

   private void removeFinalStraightStepsToBeSquared(boolean lastStepsAreStraightSteps) {
      int numSteps = stepSValues.size();
      if (lastStepsAreStraightSteps && (numSteps > 0))
      {
         // Remove last straight step since it can go directly to the correct orientation with the final square up steps
         int lastStep = numSteps - 1;
         removeSubpathStep(lastStep);
   
         // Need to remove two steps if left/right path and no turn steps taken
         if (isLeftRightMotion)
         {
            lastStep = numSteps - 2;
            removeSubpathStep(lastStep);
         }
      }
   }

   private void generateFinalSquareUpStep() {
      addSubpathStep(1.0, 1.0, getNumberOfSubPaths()-1); 
   }
}
