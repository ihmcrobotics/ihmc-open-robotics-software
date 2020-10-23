package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;

public class FootstepPlannerParameterKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   // Algorithm parameters
   public static final DoubleStoredPropertyKey  aStarHeuristicsWeight                        = keys.addDoubleKey ("AStar heuristics weight", 1.5);
   public static final IntegerStoredPropertyKey maximumBranchFactor                          = keys.addIntegerKey ("Max branch factor", -1);
   public static final BooleanStoredPropertyKey enableExpansionMask                          = keys.addBooleanKey("Enable expansion mask", true);

   // Ideal footstep parameters
   public static final DoubleStoredPropertyKey  idealFootstepWidth                           = keys.addDoubleKey ("Ideal footstep width", 0.22);
   public static final DoubleStoredPropertyKey  idealFootstepLength                          = keys.addDoubleKey ("Ideal footstep length", 0.3);
   public static final DoubleStoredPropertyKey  idealSideStepWidth                           = keys.addDoubleKey ("Ideal side step width", 0.35);
   public static final DoubleStoredPropertyKey  idealBackStepLength                          = keys.addDoubleKey ("Ideal back step length", 0.2);
   public static final DoubleStoredPropertyKey  idealStepLengthAtMaxStepZ                    = keys.addDoubleKey ("Ideal step length at max step z", 0.2);

   // Footstep restriction parameters
   public static final DoubleStoredPropertyKey  minStepWidth                                 = keys.addDoubleKey ("Min step width", 0.15);
   public static final DoubleStoredPropertyKey  minStepLength                                = keys.addDoubleKey ("Min step length", -0.25);
   public static final DoubleStoredPropertyKey  minSurfaceIncline                            = keys.addDoubleKey ("Min surface incline", Math.toRadians(45.0));
   public static final DoubleStoredPropertyKey  minStepYaw                                   = keys.addDoubleKey ("Min step yaw", -0.3);
   public static final DoubleStoredPropertyKey  minStepZWhenFullyPitched                     = keys.addDoubleKey ("Min step Z when fully pitched", 0.35);
   public static final DoubleStoredPropertyKey  minFootholdPercent                           = keys.addDoubleKey ("Min foothold percent", 0.9);
   public static final DoubleStoredPropertyKey  minClearanceFromStance                       = keys.addDoubleKey ("Min clearance from stance", 0.0);

   public static final DoubleStoredPropertyKey  maxStepWidth                                 = keys.addDoubleKey ("Max step width", 0.4);
   public static final DoubleStoredPropertyKey  maxStepReach                                 = keys.addDoubleKey ("Max step reach", 0.45);
   public static final DoubleStoredPropertyKey  maxStepYaw                                   = keys.addDoubleKey ("Max step yaw", 0.6);
   public static final DoubleStoredPropertyKey  maxStepXWhenFullyPitched                     = keys.addDoubleKey ("Max step X when fully pitched", 0.45);
   public static final DoubleStoredPropertyKey  maximumStepXWhenForwardAndDown               = keys.addDoubleKey ("Max step X when forward and down", 0.35);
   public static final DoubleStoredPropertyKey  maximumStepYWhenForwardAndDown               = keys.addDoubleKey ("Max step Y when forward and down", 0.4);
   public static final DoubleStoredPropertyKey  maximumStepZWhenForwardAndDown               = keys.addDoubleKey ("Max step Z when forward and down", 0.1);
   public static final DoubleStoredPropertyKey  maximumStepReachWhenSteppingUp               = keys.addDoubleKey ("Max step reach when stepping up", 0.45);
   public static final DoubleStoredPropertyKey  maximumStepWidthWhenSteppingUp               = keys.addDoubleKey ("Max step width when stepping up", 0.4);
   public static final DoubleStoredPropertyKey  maximumStepZWhenSteppingUp                   = keys.addDoubleKey ("Max step Z when stepping up", Double.POSITIVE_INFINITY);

   public static final DoubleStoredPropertyKey  maxStepZ                                     = keys.addDoubleKey ("Max step Z", 0.28);
   public static final DoubleStoredPropertyKey  maxSwingZ                                    = keys.addDoubleKey ("Max swing Z", 0.56);
   public static final DoubleStoredPropertyKey  maxSwingReach                                = keys.addDoubleKey ("Max swing reach", 0.9);

   public static final DoubleStoredPropertyKey  stepYawReductionFactorAtMaxReach             = keys.addDoubleKey("Step yaw reduction factor at max reach", 0.0);

   // Footstep cost parameters
   public static final DoubleStoredPropertyKey  yawWeight                                    = keys.addDoubleKey ("Yaw weight", 0.2);
   public static final DoubleStoredPropertyKey  forwardWeight                                = keys.addDoubleKey ("Forward weight", 0.2);
   public static final DoubleStoredPropertyKey  lateralWeight                                = keys.addDoubleKey ("Lateral weight", 0.2);
   public static final DoubleStoredPropertyKey  costPerStep                                  = keys.addDoubleKey ("Cost per step", 0.15);
   public static final DoubleStoredPropertyKey  stepUpWeight                                 = keys.addDoubleKey ("Step up weight", 0.2);
   public static final DoubleStoredPropertyKey  stepDownWeight                               = keys.addDoubleKey ("Step down weight", 0.2);
   public static final DoubleStoredPropertyKey  rollWeight                                   = keys.addDoubleKey ("Roll weight", 0.2);
   public static final DoubleStoredPropertyKey  pitchWeight                                  = keys.addDoubleKey ("Pitch weight", 0.2);
   public static final DoubleStoredPropertyKey  maximum2dDistanceFromBoundingBoxToPenalize   = keys.addDoubleKey ("Maximum 2D distance from bounding box to penalize", 0.0);
   public static final DoubleStoredPropertyKey  footholdAreaWeight                           = keys.addDoubleKey ("Foothold area weight", 0.0);

   // Footstep snapping and wiggling
   public static final DoubleStoredPropertyKey  wiggleInsideDeltaTarget                      = keys.addDoubleKey ("Wiggle inside delta target", 0.01);
   public static final DoubleStoredPropertyKey  wiggleInsideDeltaMinimum                     = keys.addDoubleKey ("Wiggle inside delta minimum", 0.0);
   public static final BooleanStoredPropertyKey enableConcaveHullWiggler                     = keys.addBooleanKey("Enable concave hull wiggler", false);
   public static final BooleanStoredPropertyKey wiggleWhilePlanning                          = keys.addBooleanKey("Wiggle while planning", false);
   public static final DoubleStoredPropertyKey  maximumXYWiggleDistance                      = keys.addDoubleKey ("Max XY wiggle distance", LatticePoint.gridSizeXY / 2.0);
   public static final DoubleStoredPropertyKey  maximumYawWiggle                             = keys.addDoubleKey ("Max yaw wiggle", LatticePoint.gridSizeYaw / 2.0);
   public static final DoubleStoredPropertyKey  maximumZPenetrationOnValleyRegions           = keys.addDoubleKey ("Max Z penetration on valley regions", Double.POSITIVE_INFINITY);
   public static final DoubleStoredPropertyKey  maximumSnapHeight                            = keys.addDoubleKey("Maximum snap height", 2.5);

   // Body path parameters
   public static final DoubleStoredPropertyKey  finalTurnProximity                           = keys.addDoubleKey ("Final turn proximity", 0.2);
   public static final DoubleStoredPropertyKey  distanceFromPathTolerance                    = keys.addDoubleKey("Distance from path tolerance", 0.3);
   public static final DoubleStoredPropertyKey  deltaYawFromReferenceTolerance               = keys.addDoubleKey("Delta yaw from reference tolerance", 0.35);

   // Bounding box collision check
   public static final BooleanStoredPropertyKey checkForBodyBoxCollisions                    = keys.addBooleanKey("Check for body box collisions", false);
   public static final DoubleStoredPropertyKey  bodyBoxWidth                                 = keys.addDoubleKey ("Body box width", 0.7);
   public static final DoubleStoredPropertyKey  bodyBoxHeight                                = keys.addDoubleKey ("Body box height", 1.5);
   public static final DoubleStoredPropertyKey  bodyBoxDepth                                 = keys.addDoubleKey ("Body box depth", 0.3);
   public static final DoubleStoredPropertyKey  bodyBoxBaseX                                 = keys.addDoubleKey ("Body box base X", 0.0);
   public static final DoubleStoredPropertyKey  bodyBoxBaseY                                 = keys.addDoubleKey ("Body box base Y", 0.0);
   public static final DoubleStoredPropertyKey  bodyBoxBaseZ                                 = keys.addDoubleKey ("Body box base Z", 0.25);
   public static final IntegerStoredPropertyKey numberOfBoundingBoxChecks                    = keys.addIntegerKey("Number of bounding box checks", 1);

   // Shin collision check
   public static final BooleanStoredPropertyKey enableShinCollisionCheck                     = keys.addBooleanKey("Enable shin collision check", false);
   public static final DoubleStoredPropertyKey  shinLength                                   = keys.addDoubleKey("Shin length", 0.45);
   public static final DoubleStoredPropertyKey  shinToeClearance                             = keys.addDoubleKey("Shin toe clearance", 0.0);
   public static final DoubleStoredPropertyKey  shinHeelClearance                            = keys.addDoubleKey("Shin heel clearance", 0.0);
   public static final DoubleStoredPropertyKey  shinHeightOffet                              = keys.addDoubleKey("Shin height offet", 0.05);

   // Other collision-related checks
   public static final BooleanStoredPropertyKey checkForPathCollisions                       = keys.addBooleanKey("Check for path collisions", true);
   public static final DoubleStoredPropertyKey  cliffBaseHeightToAvoid                       = keys.addDoubleKey ("Cliff bottom height to avoid", Double.MAX_VALUE);
   public static final DoubleStoredPropertyKey  minimumDistanceFromCliffBottoms              = keys.addDoubleKey ("Min distance from cliff bottoms", 0.0);
   public static final DoubleStoredPropertyKey  cliffTopHeightToAvoid                        = keys.addDoubleKey ("Cliff top height to avoid", Double.MAX_VALUE);
   public static final DoubleStoredPropertyKey  minimumDistanceFromCliffTops                 = keys.addDoubleKey ("Min distance from cliff tops", 0.0);
}
