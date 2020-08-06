package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.tools.property.*;

public class LookAndStepBehaviorParameters extends StoredPropertySet implements LookAndStepBehaviorParametersReadOnly
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
   public static final IntegerStoredPropertyKey minimumNumberOfPlannedSteps = keys.addIntegerKey("Minimum number of planned steps");
   public static final DoubleStoredPropertyKey goalSatisfactionRadius = keys.addDoubleKey("Goal satisfaction radius");
   public static final DoubleStoredPropertyKey goalSatisfactionOrientationDelta = keys.addDoubleKey("Goal satisfaction orientation delta");
   public static final DoubleStoredPropertyKey percentSwingToWait = keys.addDoubleKey("Percent swing to wait");
   public static final DoubleStoredPropertyKey robotConfigurationDataExpiration = keys.addDoubleKey("Robot configuration data expiration");
   public static final IntegerStoredPropertyKey acceptableIncompleteFootsteps = keys.addIntegerKey("Acceptable incomplete footsteps");
   public static final DoubleStoredPropertyKey minimumSwingFootClearanceOverride = keys.addDoubleKey("Minimum swing foot clearance override");
   public static final DoubleStoredPropertyKey neckPitchForBodyPath = keys.addDoubleKey("Neck pitch for body path");
   public static final DoubleStoredPropertyKey neckPitchTolerance = keys.addDoubleKey("Neck pitch tolerance");
   public static final DoubleStoredPropertyKey resetDuration = keys.addDoubleKey("Reset duration");

   public LookAndStepBehaviorParameters()
   {
      super(keys, LookAndStepBehaviorParameters.class, "ihmc-open-robotics-software", "ihmc-avatar-interfaces/src/behavior-clean-room/resources");
      load();
   }

   public static void main(String[] args)
   {
      LookAndStepBehaviorParameters parameters = new LookAndStepBehaviorParameters();
      parameters.save();
   }
}
