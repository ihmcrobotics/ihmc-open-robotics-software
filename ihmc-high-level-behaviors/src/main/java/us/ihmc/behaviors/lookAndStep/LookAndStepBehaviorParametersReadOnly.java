package us.ihmc.behaviors.lookAndStep;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorParameters.*;

public interface LookAndStepBehaviorParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean getUseInitialSupportRegions()
   {
      return get(useInitialSupportRegions);
   }

   default boolean getAssumeFlatGround()
   {
      return get(assumeFlatGround);
   }

   default boolean getDetectFlatGround()
   {
      return get(detectFlatGround);
   }

   default double getDetectFlatGroundZTolerance()
   {
      return get(detectFlatGroundZTolerance);
   }

   default double getDetectFlatGroundOrientationTolerance()
   {
      return get(detectFlatGroundOrientationTolerance);
   }

   default double getDetectFlatGroundMinRegionAreaToConsider()
   {
      return get(detectFlatGroundMinRegionAreaToConsider);
   }

   default double getDetectFlatGroundMinRadius()
   {
      return get(detectFlatGroundMinRadius);
   }

   default double getAssumedFlatGroundCircleRadius()
   {
      return get(assumedFlatGroundCircleRadius);
   }

   default boolean getSquareUpAtTheEnd()
   {
      return get(squareUpAtTheEnd);
   }

   default double getSupportRegionScaleFactor()
   {
      return get(supportRegionScaleFactor);
   }

   default int getPlanarRegionsHistorySize()
   {
      return get(planarRegionsHistorySize);
   }

   default int getMaxStepsToSendToController()
   {
      return get(maxStepsToSendToController);
   }

   default boolean getFlatGroundBodyPathPlan()
   {
      return get(flatGroundBodyPathPlan);
   }

   default int getNumberOfStepsToTryToPlan()
   {
      return get(numberOfStepsToTryToPlan);
   }

   default double getMinimumStepTranslation()
   {
      return get(minimumStepTranslation);
   }

   default double getMinimumStepOrientation()
   {
      return get(minimumStepOrientation);
   }

   default double getGoalSatisfactionRadius()
   {
      return get(goalSatisfactionRadius);
   }

   default double getGoalSatisfactionOrientationDelta()
   {
      return get(goalSatisfactionOrientationDelta);
   }

   default double getPlanarRegionsExpiration()
   {
      return get(planarRegionsExpiration);
   }

   default double getPlanHorizon()
   {
      return get(planHorizon);
   }

   default double getFootstepPlannerTimeout()
   {
      return get(footstepPlannerTimeout);
   }

   default double getSwingDuration()
   {
      return get(swingDuration);
   }

   default double getTransferDuration()
   {
      return get(transferDuration);
   }

   default double getWaitTimeAfterPlanFailed()
   {
      return get(waitTimeAfterPlanFailed);
   }

   default double getPercentSwingToWait()
   {
      return get(percentSwingToWait);
   }

   default double getRobotConfigurationDataExpiration()
   {
      return get(robotConfigurationDataExpiration);
   }

   default int getAcceptableIncompleteFootsteps()
   {
      return get(acceptableIncompleteFootsteps);
   }

   default double getNeckPitchForBodyPath()
   {
      return get(neckPitchForBodyPath);
   }

   default double getNeckPitchTolerance()
   {
      return get(neckPitchTolerance);
   }

   default double getResetDuration()
   {
      return get(resetDuration);
   }

   default int getSwingPlannerType()
   {
      return get(swingPlannerType);
   }

   default double getHorizonFromDebrisToStop()
   {
      return get(horizonFromDebrisToStop);
   }

   default boolean getStopForImpassibilities()
   {
      return get(stopForImpassibilities);
   }
}
