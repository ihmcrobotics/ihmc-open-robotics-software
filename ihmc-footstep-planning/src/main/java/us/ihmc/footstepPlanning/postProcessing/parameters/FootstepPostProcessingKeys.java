package us.ihmc.footstepPlanning.postProcessing.parameters;

import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;

public class FootstepPostProcessingKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final BooleanStoredPropertyKey positionSplitFractionProcessingEnabled  = keys.addBooleanKey("Position split fraction processing enabled", true);
   public static final DoubleStoredPropertyKey  stepHeightForLargeStepDown              = keys.addDoubleKey("Step height for large step down", 0.15);
   public static final DoubleStoredPropertyKey  largestStepDownHeight                   = keys.addDoubleKey("Largest step down height", 0.35);
   public static final DoubleStoredPropertyKey  transferSplitFractionAtFullDepth        = keys.addDoubleKey("Transfer split fraction at full depth", 0.2);
   public static final DoubleStoredPropertyKey  transferWeightDistributionAtFullDepth   = keys.addDoubleKey("Transfer weight distribution at full depth", 0.7);

   public static final BooleanStoredPropertyKey swingOverRegionsProcessingEnabled       = keys.addBooleanKey("Swing over regions processing enabled", false);
   public static final BooleanStoredPropertyKey doInitialFastApproximation              = keys.addBooleanKey("Do initial fast approximation", false);
   public static final DoubleStoredPropertyKey  minimumSwingFootClearance               = keys.addDoubleKey("Minimum swing foot clearance", 0.0);
   public static final IntegerStoredPropertyKey numberOfChecksPerSwing                  = keys.addIntegerKey("Number of checks per swing", 100);
   public static final IntegerStoredPropertyKey maximumNumberOfAdjustmentAttempts       = keys.addIntegerKey("Maximum number of adjustment attempts", 50);
   public static final DoubleStoredPropertyKey  maximumWaypointAdjustmentDistance       = keys.addDoubleKey("Maximum waypoint adjustment distance", 0.2);
   public static final DoubleStoredPropertyKey  incrementalWaypointAdjustmentDistance   = keys.addDoubleKey("Incremental waypoint adjustment distance", 0.03);
   public static final DoubleStoredPropertyKey  minimumHeightAboveFloorForCollision     = keys.addDoubleKey("Minimum height above floor for collision", 0.03);

   public static final BooleanStoredPropertyKey areaSplitFractionProcessingEnabled      = keys.addBooleanKey("Area split fraction processing enabled", true);
   public static final DoubleStoredPropertyKey  fractionLoadIfFootHasFullSupport        = keys.addDoubleKey("Fraction load if foot has full support", 0.5);
   public static final DoubleStoredPropertyKey  fractionTimeOnFootIfFootHasFullSupport  = keys.addDoubleKey("Fraction time on foot if foot has full support", 0.5);
   public static final DoubleStoredPropertyKey  fractionLoadIfOtherFootHasNoWidth       = keys.addDoubleKey("Fraction load if other foot has no width", 0.5);
   public static final DoubleStoredPropertyKey  fractionTimeOnFootIfOtherFootHasNoWidth = keys.addDoubleKey("Fraction time on foot if other foot has no width", 0.5);
}
