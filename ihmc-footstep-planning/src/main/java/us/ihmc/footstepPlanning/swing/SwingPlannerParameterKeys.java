package us.ihmc.footstepPlanning.swing;

import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;

public class SwingPlannerParameterKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   // parameters for custom proportion planner
   public static final DoubleStoredPropertyKey swingHeightIfCollisionDetected             = keys.addDoubleKey("Swing Height If Collision Detected", 0.1);
   public static final DoubleStoredPropertyKey minimumSwingTime                           = keys.addDoubleKey("Minimum Swing Time", 1.2);
   public static final DoubleStoredPropertyKey maximumSwingTime                           = keys.addDoubleKey("Maximum Swing Time", 2.4);
   public static final DoubleStoredPropertyKey footStubClearance                          = keys.addDoubleKey("Foot Stub Clearance", 0.09);
   public static final DoubleStoredPropertyKey waypointProportionShiftForStubAvoidance    = keys.addDoubleKey("Waypoint Proportion Shift For Stub Avoidance", 0.13);

   // parameters for custom position planner
   public static final BooleanStoredPropertyKey doInitialFastApproximation              = keys.addBooleanKey("Do initial fast approximation", false);
   public static final DoubleStoredPropertyKey fastApproximationLessClearance           = keys.addDoubleKey("Fast approximation less clearance", 0.05);
   public static final DoubleStoredPropertyKey  minimumSwingFootClearance               = keys.addDoubleKey("Minimum swing foot clearance", 0.0);
   public static final IntegerStoredPropertyKey numberOfChecksPerSwing                  = keys.addIntegerKey("Number of checks per swing", 100);
   public static final IntegerStoredPropertyKey maximumNumberOfAdjustmentAttempts       = keys.addIntegerKey("Maximum number of adjustment attempts", 50);
   public static final DoubleStoredPropertyKey  maximumWaypointAdjustmentDistance       = keys.addDoubleKey("Maximum waypoint adjustment distance", 0.2);
   public static final DoubleStoredPropertyKey  minimumAdjustmentIncrementDistance      = keys.addDoubleKey("Minimum adjustment increment distance", 0.03);
   public static final DoubleStoredPropertyKey  maximumAdjustmentIncrementDistance      = keys.addDoubleKey("Maximum adjustment increment distance", 0.15);
   public static final DoubleStoredPropertyKey  adjustmentIncrementDistanceGain         = keys.addDoubleKey("Adjustment increment distance gain", 0.95);
   public static final DoubleStoredPropertyKey  minimumHeightAboveFloorForCollision     = keys.addDoubleKey("Minimum height above floor for collision", 0.03);
   public static final DoubleStoredPropertyKey additionalSwingTimeIfExpanded             = keys.addDoubleKey("Additional swing time if expanded", 0.25);
}
