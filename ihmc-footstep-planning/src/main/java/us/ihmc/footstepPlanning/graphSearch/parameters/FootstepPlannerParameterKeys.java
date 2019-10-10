package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;

public class FootstepPlannerParameterKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final BooleanStoredPropertyKey checkForBodyBoxCollisions                  = keys.addBooleanKey("Check for body box collisions", false);
   public static final BooleanStoredPropertyKey checkForPathCollisions                     = keys.addBooleanKey("Check for path collisions", true);
   public static final BooleanStoredPropertyKey performHeuristicSearchPolicies             = keys.addBooleanKey("Perform heuristic search policies", true);
   public static final DoubleStoredPropertyKey idealFootstepWidth                         = keys.addDoubleKey ("Ideal footstep width", 0.22);
   public static final DoubleStoredPropertyKey idealFootstepLength                        = keys.addDoubleKey ("Ideal footstep length", 0.3);
   public static final DoubleStoredPropertyKey maxStepReach                               = keys.addDoubleKey ("Max step reach", 0.45);
   public static final DoubleStoredPropertyKey  minStepLength                              = keys.addDoubleKey ("Min step length", -0.1);
   public static final DoubleStoredPropertyKey  minStepYaw                                 = keys.addDoubleKey ("Min step yaw", 0.0);
   public static final DoubleStoredPropertyKey maxStepYaw                                 = keys.addDoubleKey ("Max step yaw", 0.4);
   public static final DoubleStoredPropertyKey  minStepWidth                               = keys.addDoubleKey ("Min step width", 0.15);
   public static final DoubleStoredPropertyKey  maxStepWidth                               = keys.addDoubleKey ("Max step width", 0.4);
   public static final DoubleStoredPropertyKey  maxStepZ                                   = keys.addDoubleKey ("Max step Z", 0.28);
   public static final DoubleStoredPropertyKey  maximumStepXWhenForwardAndDown             = keys.addDoubleKey ("Max step X when forward and down", 0.35);
   public static final DoubleStoredPropertyKey  maximumStepZWhenForwardAndDown             = keys.addDoubleKey ("Max step Z when forward and down", 0.1);
   public static final DoubleStoredPropertyKey wiggleInsideDelta                          = keys.addDoubleKey ("Wiggle inside delta", 0.01);
   public static final DoubleStoredPropertyKey  maximumStepReachWhenSteppingUp             = keys.addDoubleKey ("Max step reach when stepping up", 0.45);
   public static final DoubleStoredPropertyKey  maximumStepZWhenSteppingUp                 = keys.addDoubleKey ("Max step Z when stepping up", Double.POSITIVE_INFINITY);
   public static final DoubleStoredPropertyKey  minFootholdPercent                         = keys.addDoubleKey ("Min foothold percent", 0.9);
   public static final DoubleStoredPropertyKey  minSurfaceIncline                          = keys.addDoubleKey ("Min surface incline", Math.toDegrees(45.0));
   public static final BooleanStoredPropertyKey wiggleIntoConvexHullOfPlanarRegions        = keys.addBooleanKey("Wiggle into convex hull of planar regions", false);
   public static final BooleanStoredPropertyKey rejectIfCannotFullyWiggleInside            = keys.addBooleanKey("Reject if cannot fully wiggle inside",false);
   public static final DoubleStoredPropertyKey  maximumXYWiggleDistance                    = keys.addDoubleKey ("Max XY wiggle distance", LatticeNode.gridSizeXY / 2.0);
   public static final DoubleStoredPropertyKey  maximumYawWiggle                           = keys.addDoubleKey ("Max yaw wiggle", LatticeNode.gridSizeYaw / 2.0);
   public static final DoubleStoredPropertyKey  maximumZPenetrationOnValleyRegions         = keys.addDoubleKey ("Max Z penetration on valley regions", Double.POSITIVE_INFINITY);
   public static final DoubleStoredPropertyKey  cliffHeightToAvoid                         = keys.addDoubleKey ("Cliff height to avoid", Double.MAX_VALUE);
   public static final DoubleStoredPropertyKey  minimumDistanceFromCliffBottoms            = keys.addDoubleKey ("Min distance from cliff bottoms", 0.0);
   public static final BooleanStoredPropertyKey returnBestEffortPlan                       = keys.addBooleanKey("Return best effort plan", false);
   public static final IntegerStoredPropertyKey minimumStepsForBestEffortPlan              = keys.addIntegerKey("Min steps for best effort plan", 3);
   public static final DoubleStoredPropertyKey  minXClearanceFromStance                    = keys.addDoubleKey ("Min X clearance from stance", 0.0);
   public static final DoubleStoredPropertyKey  minYClearanceFromStance                    = keys.addDoubleKey ("Min Y clearance from stance", 0.0);
   public static final DoubleStoredPropertyKey  bodyBoxWidth                               = keys.addDoubleKey ("Body box width", 0.7);
   public static final DoubleStoredPropertyKey  bodyBoxHeight                              = keys.addDoubleKey ("Body box height", 1.5);
   public static final DoubleStoredPropertyKey  bodyBoxDepth                               = keys.addDoubleKey ("Body box depth", 0.3);
   public static final DoubleStoredPropertyKey  bodyBoxBaseX                               = keys.addDoubleKey ("Body box base X", 0.0);
   public static final DoubleStoredPropertyKey  bodyBoxBaseY                               = keys.addDoubleKey ("Body box base Y", 0.0);
   public static final DoubleStoredPropertyKey  bodyBoxBaseZ                               = keys.addDoubleKey ("Body box base Z", 0.25);
   public static final DoubleStoredPropertyKey  finalTurnProximity                         = keys.addDoubleKey ("Final turn proximity", 1.0);
   public static final DoubleStoredPropertyKey finalTurnBodyPathProximity                  = keys.addDoubleKey ("Final turn body path proximity", 0.2);
   public static final DoubleStoredPropertyKey  finalTurnProximityBlendFactor              = keys.addDoubleKey ("Final turn proximity blend factor", 0.25);
   public static final IntegerStoredPropertyKey numberOfBoundingBoxChecks                  = keys.addIntegerKey("Number of bounding box checks", 1);

   // cost parameters
   public static final BooleanStoredPropertyKey useQuadraticDistanceCost                   = keys.addBooleanKey("Use quadratic distance cost", false);
   public static final BooleanStoredPropertyKey useQuadraticHeightCost                     = keys.addBooleanKey("Use quadratic height cost", false);
   public static final DoubleStoredPropertyKey  aStarHeuristicsWeight                      = keys.addDoubleKey ("AStar heuristics weight", 1.5);
   public static final DoubleStoredPropertyKey  visGraphWithAStarHeuristicsWeight          = keys.addDoubleKey ("Vis graph with AStar heuristics weight", 15.0);
   public static final DoubleStoredPropertyKey  depthFirstHeuristicsWeight                 = keys.addDoubleKey ("Depth first heuristics weight", 1.0);
   public static final DoubleStoredPropertyKey  bodyPathBasedHeuristicsWeight              = keys.addDoubleKey ("Body path based heuristics weight", 1.0);
   public static final DoubleStoredPropertyKey  yawWeight                                  = keys.addDoubleKey ("Yaw weight", 0.1);
   public static final DoubleStoredPropertyKey  forwardWeight                              = keys.addDoubleKey ("Forward weight", 1.0);
   public static final DoubleStoredPropertyKey  lateralWeight                              = keys.addDoubleKey ("Lateral weight", 1.0);
   public static final DoubleStoredPropertyKey  costPerStep                                = keys.addDoubleKey ("Cost per step", 0.15);
   public static final DoubleStoredPropertyKey  stepUpWeight                               = keys.addDoubleKey ("Step up weight", 0.0);
   public static final DoubleStoredPropertyKey  stepDownWeight                             = keys.addDoubleKey ("Step down weight", 0.0);
   public static final DoubleStoredPropertyKey  rollWeight                                 = keys.addDoubleKey ("Roll weight", 0.0);
   public static final DoubleStoredPropertyKey  pitchWeight                                = keys.addDoubleKey ("Pitch weight", 0.0);
   public static final DoubleStoredPropertyKey  maximum2dDistanceFromBoundingBoxToPenalize = keys.addDoubleKey ("Maximum 2D distance from bounding box to penalize", 0.0);
   public static final DoubleStoredPropertyKey  boundingBoxCost                            = keys.addDoubleKey ("Bounding box cost", 0.0);
   public static final DoubleStoredPropertyKey  footholdAreaWeight                         = keys.addDoubleKey ("Foothold area weight", 0.0);
   public static final DoubleStoredPropertyKey  longStepWeight                             = keys.addDoubleKey ("Long step weight", 1.0);
   public static final DoubleStoredPropertyKey bodyPathViolationWeight                         = keys.addDoubleKey("Body path violation weight", 30.0);
   public static final DoubleStoredPropertyKey distanceFromPathTolerance                   = keys.addDoubleKey("Distance from path tolerance", 0.3);
   public static final DoubleStoredPropertyKey deltaYawFromReferenceTolerance              = keys.addDoubleKey("Delta yaw from reference tolerance", 0.2);
}
