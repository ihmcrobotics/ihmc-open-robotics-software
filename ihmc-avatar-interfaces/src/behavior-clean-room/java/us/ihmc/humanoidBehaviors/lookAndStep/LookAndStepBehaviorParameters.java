package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class LookAndStepBehaviorParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey direction = keys.addDoubleKey("Direction");
   public static final DoubleStoredPropertyKey planHorizon = keys.addDoubleKey("Plan horizon");
   public static final DoubleStoredPropertyKey idealFootstepLengthOverride = keys.addDoubleKey("Ideal footstep length override");
   public static final DoubleStoredPropertyKey wiggleInsideDeltaOverride = keys.addDoubleKey("Wiggle inside delta override");
   public static final DoubleStoredPropertyKey cliffBaseHeightToAvoidOverride = keys.addDoubleKey("Cliff base height to avoid override");
   public static final BooleanStoredPropertyKey enableConcaveHullWigglerOverride = keys.addBooleanKey("Enable concave hull wiggler override");
   public static final DoubleStoredPropertyKey footstepPlannerTimeout = keys.addDoubleKey("Footstep planner timeout");
   public static final DoubleStoredPropertyKey planarRegionsExpiration = keys.addDoubleKey("Planar regions expiration");
   public static final DoubleStoredPropertyKey swingTime = keys.addDoubleKey("Swing time");
   public static final DoubleStoredPropertyKey transferTime = keys.addDoubleKey("Transfer time");
   public static final DoubleStoredPropertyKey waitTimeAfterPlanFailed = keys.addDoubleKey("Wait time after plan failed");
   public static final BooleanStoredPropertyKey returnBestEffortPlanOverride = keys.addBooleanKey("Return best effort plan override");
   public static final DoubleStoredPropertyKey maxPlanStrayDistance = keys.addDoubleKey("Max plan stray distance");
   public static final DoubleStoredPropertyKey goalSatisfactionRadius = keys.addDoubleKey("Goal satisfaction radius");
   public static final DoubleStoredPropertyKey percentSwingToWait = keys.addDoubleKey("Percent swing to wait");

   public LookAndStepBehaviorParameters()
   {
      super(keys, LookAndStepBehaviorParameters.class, "ihmc-open-robotics-software", "ihmc-avatar-interfaces/src/behavior-clean-room/resources");
      load();
   }

   public double getMaxPlanStrayDistance()
   {
      return get(maxPlanStrayDistance);
   }

   public double getGoalSatisfactionRadius()
   {
      return get(goalSatisfactionRadius);
   }

   public double getPlanarRegionsExpiration()
   {
      return get(planarRegionsExpiration);
   }

   public double getDirection()
   {
      return get(direction);
   }

   public double getWiggleInsideDeltaOverride()
   {
      return get(wiggleInsideDeltaOverride);
   }

   public double getPlanHorizon()
   {
      return get(planHorizon);
   }

   public double getIdealFootstepLengthOverride()
   {
      return get(idealFootstepLengthOverride);
   }

   public double getCliffBaseHeightToAvoidOverride()
   {
      return get(cliffBaseHeightToAvoidOverride);
   }

   public boolean getEnableConcaveHullWigglerOverride()
   {
      return get(enableConcaveHullWigglerOverride);
   }

   public double getFootstepPlannerTimeout()
   {
      return get(footstepPlannerTimeout);
   }

   public double getSwingTime()
   {
      return get(swingTime);
   }

   public double getTransferTime()
   {
      return get(transferTime);
   }

   public double getWaitTimeAfterPlanFailed()
   {
      return get(waitTimeAfterPlanFailed);
   }

   public boolean getReturnBestEffortPlanOverride()
   {
      return get(returnBestEffortPlanOverride);
   }

   public double getPercentSwingToWait()
   {
      return get(percentSwingToWait);
   }

   public static void main(String[] args)
   {
      LookAndStepBehaviorParameters parameters = new LookAndStepBehaviorParameters();
      parameters.save();
   }
}
