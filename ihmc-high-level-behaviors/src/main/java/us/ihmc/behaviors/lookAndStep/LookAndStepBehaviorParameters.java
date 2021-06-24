package us.ihmc.behaviors.lookAndStep;

import us.ihmc.tools.property.*;

public class LookAndStepBehaviorParameters extends StoredPropertySet implements LookAndStepBehaviorParametersReadOnly
{
   public static final String PROJECT_NAME = "ihmc-open-robotics-software";
   public static final String TO_RESOURCE_FOLDER = "ihmc-high-level-behaviors/src/main/resources";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final BooleanStoredPropertyKey useInitialSupportRegions = keys.addBooleanKey("Use initial support regions");
   public static final DoubleStoredPropertyKey supportRegionScaleFactor = keys.addDoubleKey("Support region scale factor");
   public static final IntegerStoredPropertyKey planarRegionsHistorySize = keys.addIntegerKey("Planar regions history size");
   public static final BooleanStoredPropertyKey flatGroundBodyPathPlan = keys.addBooleanKey("Flat ground body path plan");
   public static final IntegerStoredPropertyKey swingPlannerType = keys.addIntegerKey("Swing planner type");
   public static final DoubleStoredPropertyKey minimumStepTranslation = keys.addDoubleKey("Minimum step translation");
   public static final DoubleStoredPropertyKey minimumStepOrientation = keys.addDoubleKey("Minimum step orientation");
   public static final DoubleStoredPropertyKey neckPitchForBodyPath = keys.addDoubleKey("Neck pitch for body path");
   public static final DoubleStoredPropertyKey neckPitchTolerance = keys.addDoubleKey("Neck pitch tolerance");
   public static final DoubleStoredPropertyKey percentSwingToWait = keys.addDoubleKey("Percent swing to wait");
   public static final DoubleStoredPropertyKey swingTime = keys.addDoubleKey("Swing time");
   public static final DoubleStoredPropertyKey transferTime = keys.addDoubleKey("Transfer time");
   public static final DoubleStoredPropertyKey resetDuration = keys.addDoubleKey("Reset duration");
   public static final DoubleStoredPropertyKey goalSatisfactionRadius = keys.addDoubleKey("Goal satisfaction radius");
   public static final DoubleStoredPropertyKey goalSatisfactionOrientationDelta = keys.addDoubleKey("Goal satisfaction orientation delta");
   public static final DoubleStoredPropertyKey planHorizon = keys.addDoubleKey("Plan horizon");
   public static final DoubleStoredPropertyKey footstepPlannerTimeout = keys.addDoubleKey("Footstep planner timeout");
   public static final DoubleStoredPropertyKey planarRegionsExpiration = keys.addDoubleKey("Planar regions expiration");
   public static final DoubleStoredPropertyKey waitTimeAfterPlanFailed = keys.addDoubleKey("Wait time after plan failed");
   public static final IntegerStoredPropertyKey minimumNumberOfPlannedSteps = keys.addIntegerKey("Minimum number of planned steps");
   public static final DoubleStoredPropertyKey robotConfigurationDataExpiration = keys.addDoubleKey("Robot configuration data expiration");
   public static final IntegerStoredPropertyKey acceptableIncompleteFootsteps = keys.addIntegerKey("Acceptable incomplete footsteps");
   public static final DoubleStoredPropertyKey horizonFromDebrisToStop = keys.addDoubleKey("Horizon from debris to stop");
   public static final BooleanStoredPropertyKey stopForImpassibilities = keys.addBooleanKey("Stop for impassibilities");

   public LookAndStepBehaviorParameters()
   {
      super(keys, LookAndStepBehaviorParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
      load();
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys, LookAndStepBehaviorParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
      parameters.loadUnsafe();
      parameters.save();
   }
}
