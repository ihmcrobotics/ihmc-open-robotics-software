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

   // parameters for custom two-waypoint position planner
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

   // parameters for custom multi-waypoint position planner
   public static final DoubleStoredPropertyKey percentageExtraSizeXLow = keys.addDoubleKey("Percentage extra size x low", 0.1);
   public static final DoubleStoredPropertyKey percentageExtraSizeXHigh = keys.addDoubleKey("Percentage extra size x high", 0.25);
   public static final DoubleStoredPropertyKey extraSizeXLow = keys.addDoubleKey("Extra size x low", 0.05);
   public static final DoubleStoredPropertyKey extraSizeXHigh = keys.addDoubleKey("Extra size x high", 0.11);
   public static final DoubleStoredPropertyKey percentageExtraSizeYLow = keys.addDoubleKey("Percentage extra size y low", 0.1);
   public static final DoubleStoredPropertyKey percentageExtraSizeYHigh = keys.addDoubleKey("Percentage extra size y high", 0.3);
   public static final DoubleStoredPropertyKey extraSizeYLow = keys.addDoubleKey("Extra size y low", 0.03);
   public static final DoubleStoredPropertyKey extraSizeYHigh = keys.addDoubleKey("Extra size y high", 0.07);
   public static final DoubleStoredPropertyKey percentageExtraSizeZLow = keys.addDoubleKey("Percentage extra size z low", 0.15);
   public static final DoubleStoredPropertyKey percentageExtraSizeZHigh = keys.addDoubleKey("Percentage extra size z high", 0.3);
   public static final DoubleStoredPropertyKey extraSizeZLow = keys.addDoubleKey("Extra size z low", -0.005);
   public static final DoubleStoredPropertyKey extraSizeZHigh = keys.addDoubleKey("Extra size z high", 0.06);
   public static final DoubleStoredPropertyKey percentageMaxDisplacementLow = keys.addDoubleKey("Percentage max displacement low", 0.05);
   public static final DoubleStoredPropertyKey percentageMaxDisplacementHigh = keys.addDoubleKey("Percentage max displacement high", 0.3);
   public static final DoubleStoredPropertyKey maxDisplacementLow = keys.addDoubleKey("Max displacement low", 0.05);
   public static final DoubleStoredPropertyKey maxDisplacementHigh = keys.addDoubleKey("Max displacement high", 0.1);
   public static final DoubleStoredPropertyKey motionCorrelationAlpha = keys.addDoubleKey("Motion correlation alpha", 0.65);
   public static final BooleanStoredPropertyKey allowLateralMotion = keys.addBooleanKey("Allow lateral motion", true);
   public static final DoubleStoredPropertyKey minXYTranslationToPlanSwing = keys.addDoubleKey("Min XY translation to plan swing", 0.15);
}
