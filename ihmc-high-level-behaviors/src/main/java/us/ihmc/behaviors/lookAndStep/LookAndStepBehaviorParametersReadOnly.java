package us.ihmc.behaviors.lookAndStep;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorParameters.*;

public interface LookAndStepBehaviorParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean getUseInitialSupportRegions()
   {
      return get(useInitialSupportRegions);
   }

   default double getSupportRegionScaleFactor()
   {
      return get(supportRegionScaleFactor);
   }

   default int getPlanarRegionsHistorySize()
   {
      return get(planarRegionsHistorySize);
   }

   default boolean getFlatGroundBodyPathPlan()
   {
      return get(flatGroundBodyPathPlan);
   }

   default int getMinimumNumberOfPlannedSteps()
   {
      return get(minimumNumberOfPlannedSteps);
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

   default double getSwingTime()
   {
      return get(swingTime);
   }

   default double getTransferTime()
   {
      return get(transferTime);
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
