package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters;

import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;

public class PawStepPlannerParameterKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey maximumFrontStepReach                   = keys.addDoubleKey("Maximum front step reach", 0.5);
   public static final DoubleStoredPropertyKey maximumFrontStepLength                  = keys.addDoubleKey("Maximum front step length", 0.45);
   public static final DoubleStoredPropertyKey minimumFrontStepLength                  = keys.addDoubleKey("Minimum front step length", -0.45);
   public static final DoubleStoredPropertyKey maximumHindStepReach                    = keys.addDoubleKey("Maximum hind step reach", 0.5);
   public static final DoubleStoredPropertyKey maximumHindStepLength                   = keys.addDoubleKey("Maximum hind step length", 0.45);
   public static final DoubleStoredPropertyKey minimumHindStepLength                   = keys.addDoubleKey("Minimum hind step length", -0.45);
   public static final DoubleStoredPropertyKey maximumStepOutward                      = keys.addDoubleKey("Maximum step outward", 0.3);
   public static final DoubleStoredPropertyKey maximumStepInward                       = keys.addDoubleKey("Maximum step inward", -0.15);

   public static final DoubleStoredPropertyKey maximumFrontStepLengthWhenSteppingUp    = keys.addDoubleKey("Maximum front step length when stepping up", 0.45);
   public static final DoubleStoredPropertyKey minimumFrontStepLengthWhenSteppingUp    = keys.addDoubleKey("Minimum front step length when stepping up", -0.45);
   public static final DoubleStoredPropertyKey maximumHindStepLengthWhenSteppingUp     = keys.addDoubleKey("Maximum hind step length when stepping up", 0.45);
   public static final DoubleStoredPropertyKey minimumHindStepLengthWhenSteppingUp     = keys.addDoubleKey("Minimum hind step length when stepping up", -0.45);
   public static final DoubleStoredPropertyKey stepZForSteppingUp                      = keys.addDoubleKey("Step Z for stepping up", Double.POSITIVE_INFINITY);
   public static final DoubleStoredPropertyKey maximumFrontStepLengthWhenSteppingDown  = keys.addDoubleKey("Maximum front step length when stepping down", 0.45);
   public static final DoubleStoredPropertyKey minimumFrontStepLengthWhenSteppingDown  = keys.addDoubleKey("Minimum front step length when stepping down", -0.45);
   public static final DoubleStoredPropertyKey maximumHindStepLengthWhenSteppingDown   = keys.addDoubleKey("Maximum hind step length when stepping down", 0.45);
   public static final DoubleStoredPropertyKey minimumHindStepLengthWhenSteppingDown   = keys.addDoubleKey("Minimum hind step length when stepping down", -0.45);
   public static final DoubleStoredPropertyKey stepZForSteppingDown                    = keys.addDoubleKey("Step Z for stepping down", Double.NEGATIVE_INFINITY);

   public static final DoubleStoredPropertyKey maximumStepYawInward                    = keys.addDoubleKey("Maximum step yaw inward", -0.2);
   public static final DoubleStoredPropertyKey maximumStepYawOutward                   = keys.addDoubleKey("Maximum step yaw outward", 0.5);
   public static final DoubleStoredPropertyKey maximumStepChangeZ                      = keys.addDoubleKey("Maximum step change z", 0.35);
   public static final DoubleStoredPropertyKey bodyGroundClearance                     = keys.addDoubleKey("Body ground clearance", 0.35);

   public static final DoubleStoredPropertyKey distanceWeight                          = keys.addDoubleKey("Distance weight", 1.0);
   public static final DoubleStoredPropertyKey yawWeight                               = keys.addDoubleKey("Yaw weight", 2.5);
   public static final DoubleStoredPropertyKey xGaitWeight                             = keys.addDoubleKey("X gait weight", 7.5);
   public static final DoubleStoredPropertyKey desiredVelocityWeight                   = keys.addDoubleKey("Desired velocity weight", 1.0);
   public static final DoubleStoredPropertyKey costPerStep                             = keys.addDoubleKey("Cost per step", 0.25);
   public static final DoubleStoredPropertyKey stepUpWeight                            = keys.addDoubleKey("Step up weight", 0.0);
   public static final DoubleStoredPropertyKey stepDownWeight                          = keys.addDoubleKey("Step down weight", 0.0);
   public static final DoubleStoredPropertyKey heuristicsInflationWeight               = keys.addDoubleKey("Heuristics inflation weight", 1.75);

   public static final DoubleStoredPropertyKey minXClearanceFromPaw                    = keys.addDoubleKey("Min x clearance from paw", 0.05);
   public static final DoubleStoredPropertyKey minYClearanceFromPaw                    = keys.addDoubleKey("Min y clearance from paw", 0.05);
   public static final DoubleStoredPropertyKey maxWalkingSpeedMultiplier               = keys.addDoubleKey("Max walking speed multiplier", 0.8);
   public static final DoubleStoredPropertyKey projectInsideDistance                   = keys.addDoubleKey("Project inside distance", 0.02);
   public static final DoubleStoredPropertyKey minimumProjectInsideDistance            = keys.addDoubleKey("Minimum project inside distance", 0.01);
   public static final BooleanStoredPropertyKey projectInsideUsingConvexHull           = keys.addBooleanKey("Project inside using convex hull", true);

   public static final DoubleStoredPropertyKey maximumXYWiggleDistance                 = keys.addDoubleKey("Maximum xy wiggle distance", 0.03);
   public static final DoubleStoredPropertyKey minimumSurfaceInclineRadians            = keys.addDoubleKey("Minimum surface incline", Math.toRadians(45.0));
   public static final DoubleStoredPropertyKey cliffHeightToAvoid                      = keys.addDoubleKey("Cliff height to avoid", 0.15);

   public static final DoubleStoredPropertyKey minimumFrontEndForwardDistanceFromCliffBottoms  = keys.addDoubleKey("Min front end forward distance from cliff bottoms", 0.1);
   public static final DoubleStoredPropertyKey minimumFrontEndBackwardDistanceFromCliffBottoms = keys.addDoubleKey("Min front end backward distance from cliff bottoms", 0.1);
   public static final DoubleStoredPropertyKey minimumHindEndForwardDistanceFromCliffBottoms   = keys.addDoubleKey("Min hind end forward distance from cliff bottoms", 0.1);
   public static final DoubleStoredPropertyKey minimumHindEndBackwardDistanceFromCliffBottoms  = keys.addDoubleKey("Min hind end backward distance from cliff bottoms", 0.1);
   public static final DoubleStoredPropertyKey minimumLateralDistanceFromCliffBottoms          = keys.addDoubleKey("Min lateral distance from cliff bottoms", 0.1);

   public static final DoubleStoredPropertyKey finalTurnProximity                       = keys.addDoubleKey("Final turn proximity", 1.0);
   public static final DoubleStoredPropertyKey finalSlowDownProximity                   = keys.addDoubleKey("Final slow down proximity", 0.5);
   public static final DoubleStoredPropertyKey maximumDeviationFromXGaitDuringExpansion = keys.addDoubleKey("Max deviation from x gait during expansion", 0.1);

   public static final BooleanStoredPropertyKey returnBestEffortPlan                    = keys.addBooleanKey("Return best effort plan", false);
   public static final IntegerStoredPropertyKey minStepsForBestEffortPlan               = keys.addIntegerKey("Min steps for best effort plan", 4);
   public static final BooleanStoredPropertyKey performGraphRepairingStep               = keys.addBooleanKey("Perform graph repairing step", false);
   public static final DoubleStoredPropertyKey repairingHeuristicWeightScaling          = keys.addDoubleKey("Repairing heuristic weight scaling", 0.9);
   public static final DoubleStoredPropertyKey minHeuristicWeightReduction              = keys.addDoubleKey("Min heuristic weight reduction", 0.2);
}
