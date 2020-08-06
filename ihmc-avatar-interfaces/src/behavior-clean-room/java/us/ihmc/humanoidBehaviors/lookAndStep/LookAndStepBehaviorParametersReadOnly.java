package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

public interface LookAndStepBehaviorParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getMaxPlanStrayDistance()
   {
      return get(LookAndStepBehaviorParameters.maxPlanStrayDistance);
   }

   default int getMinimumNumberOfPlannedSteps()
   {
      return get(LookAndStepBehaviorParameters.minimumNumberOfPlannedSteps);
   }

   default double getGoalSatisfactionRadius()
   {
      return get(LookAndStepBehaviorParameters.goalSatisfactionRadius);
   }

   default double getGoalSatisfactionOrientationDelta()
   {
      return get(LookAndStepBehaviorParameters.goalSatisfactionOrientationDelta);
   }

   default double getPlanarRegionsExpiration()
   {
      return get(LookAndStepBehaviorParameters.planarRegionsExpiration);
   }

   default double getDirection()
   {
      return get(LookAndStepBehaviorParameters.direction);
   }

   default double getWiggleInsideDeltaOverride()
   {
      return get(LookAndStepBehaviorParameters.wiggleInsideDeltaOverride);
   }

   default double getPlanHorizon()
   {
      return get(LookAndStepBehaviorParameters.planHorizon);
   }

   default double getIdealFootstepLengthOverride()
   {
      return get(LookAndStepBehaviorParameters.idealFootstepLengthOverride);
   }

   default double getCliffBaseHeightToAvoidOverride()
   {
      return get(LookAndStepBehaviorParameters.cliffBaseHeightToAvoidOverride);
   }

   default boolean getEnableConcaveHullWigglerOverride()
   {
      return get(LookAndStepBehaviorParameters.enableConcaveHullWigglerOverride);
   }

   default double getFootstepPlannerTimeout()
   {
      return get(LookAndStepBehaviorParameters.footstepPlannerTimeout);
   }

   default double getSwingTime()
   {
      return get(LookAndStepBehaviorParameters.swingTime);
   }

   default double getTransferTime()
   {
      return get(LookAndStepBehaviorParameters.transferTime);
   }

   default double getWaitTimeAfterPlanFailed()
   {
      return get(LookAndStepBehaviorParameters.waitTimeAfterPlanFailed);
   }

   default boolean getReturnBestEffortPlanOverride()
   {
      return get(LookAndStepBehaviorParameters.returnBestEffortPlanOverride);
   }

   default double getPercentSwingToWait()
   {
      return get(LookAndStepBehaviorParameters.percentSwingToWait);
   }

   default double getRobotConfigurationDataExpiration()
   {
      return get(LookAndStepBehaviorParameters.robotConfigurationDataExpiration);
   }

   default int getAcceptableIncompleteFootsteps()
   {
      return get(LookAndStepBehaviorParameters.acceptableIncompleteFootsteps);
   }

   default double getMinimumSwingFootClearanceOverride()
   {
      return get(LookAndStepBehaviorParameters.minimumSwingFootClearanceOverride);
   }

   default double getNeckPitchForBodyPath()
   {
      return get(LookAndStepBehaviorParameters.neckPitchForBodyPath);
   }

   default double getNeckPitchTolerance()
   {
      return get(LookAndStepBehaviorParameters.neckPitchTolerance);
   }

   default double getResetDuration()
   {
      return get(LookAndStepBehaviorParameters.resetDuration);
   }
}
