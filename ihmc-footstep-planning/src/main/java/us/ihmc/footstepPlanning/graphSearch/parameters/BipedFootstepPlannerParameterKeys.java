package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.log.LogTools;

public class BipedFootstepPlannerParameterKeys
{
   public static final FootstepPlannerParameterKeyMap keyMap = new FootstepPlannerParameterKeyMap("bipedFootstepPlannerParameters");

   public static final BooleanFootstepPlannerParameterKey wiggleIntoConvexHullOfPlanarRegions = keyMap.addBooleanKey("Wiggle into convex hull of planar regions" );
   public static final BooleanFootstepPlannerParameterKey rejectIfCannotFullyWiggleInside     = keyMap.addBooleanKey("Reject if cannot fully wiggle inside"      );
   public static final BooleanFootstepPlannerParameterKey returnBestEffortPlan                = keyMap.addBooleanKey("Return best effort plan"                   );
   public static final BooleanFootstepPlannerParameterKey checkForBodyBoxCollisions           = keyMap.addBooleanKey("Check for body box collisions"             );
   public static final BooleanFootstepPlannerParameterKey performHeuristicSearchPolicies      = keyMap.addBooleanKey("Perform heuristic search policies"         );
   public static final IntegerFootstepPlannerParameterKey minimumStepsForBestEffortPlan       = keyMap.addIntegerKey("Min steps for best effort plan"            );
   public static final DoubleFootstepPlannerParameterKey  idealFootstepWidth                  = keyMap.addDoubleKey ("Ideal footstep width"                      );
   public static final DoubleFootstepPlannerParameterKey  idealFootstepLength                 = keyMap.addDoubleKey ("Ideal footstep length"                     );
   public static final DoubleFootstepPlannerParameterKey  wiggleInsideDelta                   = keyMap.addDoubleKey ("Wiggle inside delta"                       );
   public static final DoubleFootstepPlannerParameterKey  maximumXYWiggleDistance             = keyMap.addDoubleKey ("Max XY wiggle distance"                    );
   public static final DoubleFootstepPlannerParameterKey  maximumYawWiggle                    = keyMap.addDoubleKey ("Max yaw wiggle"                            );
   public static final DoubleFootstepPlannerParameterKey  maxStepReach                        = keyMap.addDoubleKey ("Max step reach"                            );
   public static final DoubleFootstepPlannerParameterKey  maxStepYaw                          = keyMap.addDoubleKey ("Max step yaw"                              );
   public static final DoubleFootstepPlannerParameterKey  minStepWidth                        = keyMap.addDoubleKey ("Min step width"                            );
   public static final DoubleFootstepPlannerParameterKey  minStepLength                       = keyMap.addDoubleKey ("Min step length"                           );
   public static final DoubleFootstepPlannerParameterKey  minStepYaw                          = keyMap.addDoubleKey ("Min step yaw"                              );
   public static final DoubleFootstepPlannerParameterKey  maxStepZ                            = keyMap.addDoubleKey ("Max step Z"                                );
   public static final DoubleFootstepPlannerParameterKey  minFootholdPercent                  = keyMap.addDoubleKey ("Min foothold percent"                      );
   public static final DoubleFootstepPlannerParameterKey  minSurfaceIncline                   = keyMap.addDoubleKey ("Min surface incline"                       );
   public static final DoubleFootstepPlannerParameterKey  maxStepWidth                        = keyMap.addDoubleKey ("Max step width"                            );
   public static final DoubleFootstepPlannerParameterKey  minXClearanceFromStance             = keyMap.addDoubleKey ("Min X clearance from stance"               );
   public static final DoubleFootstepPlannerParameterKey  minYClearanceFromStance             = keyMap.addDoubleKey ("Min Y clearance from stance"               );
   public static final DoubleFootstepPlannerParameterKey  maximumStepReachWhenSteppingUp      = keyMap.addDoubleKey ("Max step reach when stepping up"           );
   public static final DoubleFootstepPlannerParameterKey  maximumStepZWhenSteppingUp          = keyMap.addDoubleKey ("Max step Z when stepping up"               );
   public static final DoubleFootstepPlannerParameterKey  maximumStepXWhenForwardAndDown      = keyMap.addDoubleKey ("Max step X when forward and down"          );
   public static final DoubleFootstepPlannerParameterKey  maximumStepZWhenForwardAndDown      = keyMap.addDoubleKey ("Max step Z when forward and down"          );
   public static final DoubleFootstepPlannerParameterKey  maximumZPenetrationOnValleyRegions  = keyMap.addDoubleKey ("Max Z penetration on valley regions"       );
   public static final DoubleFootstepPlannerParameterKey  cliffHeightToAvoid                  = keyMap.addDoubleKey ("Cliff height to avoid"                     );
   public static final DoubleFootstepPlannerParameterKey  minimumDistanceFromCliffBottoms     = keyMap.addDoubleKey ("Min distance from cliff bottoms"           );
   public static final DoubleFootstepPlannerParameterKey  bodyGroundClearance                 = keyMap.addDoubleKey ("Body ground clearance"                     );
   public static final DoubleFootstepPlannerParameterKey  bodyBoxWidth                        = keyMap.addDoubleKey ("Body box width"                            );
   public static final DoubleFootstepPlannerParameterKey  bodyBoxHeight                       = keyMap.addDoubleKey ("Body box height"                           );
   public static final DoubleFootstepPlannerParameterKey  bodyBoxDepth                        = keyMap.addDoubleKey ("Body box depth"                            );
   public static final DoubleFootstepPlannerParameterKey  bodyBoxBaseX                        = keyMap.addDoubleKey ("Body box base X"                           );
   public static final DoubleFootstepPlannerParameterKey  bodyBoxBaseY                        = keyMap.addDoubleKey ("Body box base Y"                           );
   public static final DoubleFootstepPlannerParameterKey  bodyBoxBaseZ                        = keyMap.addDoubleKey ("Body box base Z"                           );



   public static void main(String[] args)
   {
      FootstepPlannerParameterMap map = new FootstepPlannerParameterMap(keyMap);
      map.printInitializedSaveFile();

      LogTools.info("{}", map.getValue(wiggleIntoConvexHullOfPlanarRegions));
      LogTools.info("{}", map.getValue(rejectIfCannotFullyWiggleInside    ));
      LogTools.info("{}", map.getValue(returnBestEffortPlan               ));
      LogTools.info("{}", map.getValue(checkForBodyBoxCollisions          ));
      LogTools.info("{}", map.getValue(performHeuristicSearchPolicies     ));
      LogTools.info("{}", map.getValue(minimumStepsForBestEffortPlan      ));
      LogTools.info("{}", map.getValue(idealFootstepWidth                 ));
      LogTools.info("{}", map.getValue(idealFootstepLength                ));
      LogTools.info("{}", map.getValue(wiggleInsideDelta                  ));
      LogTools.info("{}", map.getValue(maximumXYWiggleDistance            ));
      LogTools.info("{}", map.getValue(maximumYawWiggle                   ));
      LogTools.info("{}", map.getValue(maxStepReach                       ));
      LogTools.info("{}", map.getValue(maxStepYaw                         ));
      LogTools.info("{}", map.getValue(minStepWidth                       ));
      LogTools.info("{}", map.getValue(minStepLength                      ));
      LogTools.info("{}", map.getValue(minStepYaw                         ));
      LogTools.info("{}", map.getValue(maxStepZ                           ));
      LogTools.info("{}", map.getValue(minFootholdPercent                 ));
      LogTools.info("{}", map.getValue(minSurfaceIncline                  ));
      LogTools.info("{}", map.getValue(maxStepWidth                       ));
      LogTools.info("{}", map.getValue(minXClearanceFromStance            ));
      LogTools.info("{}", map.getValue(minYClearanceFromStance            ));
      LogTools.info("{}", map.getValue(maximumStepReachWhenSteppingUp     ));
      LogTools.info("{}", map.getValue(maximumStepZWhenSteppingUp         ));
      LogTools.info("{}", map.getValue(maximumStepXWhenForwardAndDown     ));
      LogTools.info("{}", map.getValue(maximumStepZWhenForwardAndDown     ));
      LogTools.info("{}", map.getValue(maximumZPenetrationOnValleyRegions ));
      LogTools.info("{}", map.getValue(cliffHeightToAvoid                 ));
      LogTools.info("{}", map.getValue(minimumDistanceFromCliffBottoms    ));
      LogTools.info("{}", map.getValue(bodyGroundClearance                ));
      LogTools.info("{}", map.getValue(bodyBoxWidth                       ));
      LogTools.info("{}", map.getValue(bodyBoxHeight                      ));
      LogTools.info("{}", map.getValue(bodyBoxDepth                       ));
      LogTools.info("{}", map.getValue(bodyBoxBaseX                       ));
      LogTools.info("{}", map.getValue(bodyBoxBaseY                       ));
      LogTools.info("{}", map.getValue(bodyBoxBaseZ                       ));

   }
}
