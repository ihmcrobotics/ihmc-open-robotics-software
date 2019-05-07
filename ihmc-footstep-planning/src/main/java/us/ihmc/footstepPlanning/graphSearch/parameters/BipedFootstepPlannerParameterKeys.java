package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.log.LogTools;

public class BipedFootstepPlannerParameterKeys
{
   public static final FootstepPlannerParameterKeys keys = new FootstepPlannerParameterKeys("bipedFootstepPlannerParameters");

   public static final BooleanFootstepPlannerParameterKey wiggleIntoConvexHullOfPlanarRegions = keys.addBooleanKey("Wiggle into convex hull of planar regions" );
   public static final BooleanFootstepPlannerParameterKey rejectIfCannotFullyWiggleInside     = keys.addBooleanKey("Reject if cannot fully wiggle inside"      );
   public static final BooleanFootstepPlannerParameterKey returnBestEffortPlan                = keys.addBooleanKey("Return best effort plan"                   );
   public static final BooleanFootstepPlannerParameterKey checkForBodyBoxCollisions           = keys.addBooleanKey("Check for body box collisions"             );
   public static final BooleanFootstepPlannerParameterKey performHeuristicSearchPolicies      = keys.addBooleanKey("Perform heuristic search policies"         );
   public static final IntegerFootstepPlannerParameterKey minimumStepsForBestEffortPlan       = keys.addIntegerKey("Min steps for best effort plan"            );
   public static final DoubleFootstepPlannerParameterKey  idealFootstepWidth                  = keys.addDoubleKey ("Ideal footstep width"                      );
   public static final DoubleFootstepPlannerParameterKey  idealFootstepLength                 = keys.addDoubleKey ("Ideal footstep length"                     );
   public static final DoubleFootstepPlannerParameterKey  wiggleInsideDelta                   = keys.addDoubleKey ("Wiggle inside delta"                       );
   public static final DoubleFootstepPlannerParameterKey  maximumXYWiggleDistance             = keys.addDoubleKey ("Max XY wiggle distance"                    );
   public static final DoubleFootstepPlannerParameterKey  maximumYawWiggle                    = keys.addDoubleKey ("Max yaw wiggle"                            );
   public static final DoubleFootstepPlannerParameterKey  maxStepReach                        = keys.addDoubleKey ("Max step reach"                            );
   public static final DoubleFootstepPlannerParameterKey  maxStepYaw                          = keys.addDoubleKey ("Max step yaw"                              );
   public static final DoubleFootstepPlannerParameterKey  minStepWidth                        = keys.addDoubleKey ("Min step width"                            );
   public static final DoubleFootstepPlannerParameterKey  minStepLength                       = keys.addDoubleKey ("Min step length"                           );
   public static final DoubleFootstepPlannerParameterKey  minStepYaw                          = keys.addDoubleKey ("Min step yaw"                              );
   public static final DoubleFootstepPlannerParameterKey  maxStepZ                            = keys.addDoubleKey ("Max step Z"                                );
   public static final DoubleFootstepPlannerParameterKey  minFootholdPercent                  = keys.addDoubleKey ("Min foothold percent"                      );
   public static final DoubleFootstepPlannerParameterKey  minSurfaceIncline                   = keys.addDoubleKey ("Min surface incline"                       );
   public static final DoubleFootstepPlannerParameterKey  maxStepWidth                        = keys.addDoubleKey ("Max step width"                            );
   public static final DoubleFootstepPlannerParameterKey  minXClearanceFromStance             = keys.addDoubleKey ("Min X clearance from stance"               );
   public static final DoubleFootstepPlannerParameterKey  minYClearanceFromStance             = keys.addDoubleKey ("Min Y clearance from stance"               );
   public static final DoubleFootstepPlannerParameterKey  maximumStepReachWhenSteppingUp      = keys.addDoubleKey ("Max step reach when stepping up"           );
   public static final DoubleFootstepPlannerParameterKey  maximumStepZWhenSteppingUp          = keys.addDoubleKey ("Max step Z when stepping up"               );
   public static final DoubleFootstepPlannerParameterKey  maximumStepXWhenForwardAndDown      = keys.addDoubleKey ("Max step X when forward and down"          );
   public static final DoubleFootstepPlannerParameterKey  maximumStepZWhenForwardAndDown      = keys.addDoubleKey ("Max step Z when forward and down"          );
   public static final DoubleFootstepPlannerParameterKey  maximumZPenetrationOnValleyRegions  = keys.addDoubleKey ("Max Z penetration on valley regions"       );
   public static final DoubleFootstepPlannerParameterKey  cliffHeightToAvoid                  = keys.addDoubleKey ("Cliff height to avoid"                     );
   public static final DoubleFootstepPlannerParameterKey  minimumDistanceFromCliffBottoms     = keys.addDoubleKey ("Min distance from cliff bottoms"           );
   public static final DoubleFootstepPlannerParameterKey  bodyGroundClearance                 = keys.addDoubleKey ("Body ground clearance"                     );
   public static final DoubleFootstepPlannerParameterKey  bodyBoxWidth                        = keys.addDoubleKey ("Body box width"                            );
   public static final DoubleFootstepPlannerParameterKey  bodyBoxHeight                       = keys.addDoubleKey ("Body box height"                           );
   public static final DoubleFootstepPlannerParameterKey  bodyBoxDepth                        = keys.addDoubleKey ("Body box depth"                            );
   public static final DoubleFootstepPlannerParameterKey  bodyBoxBaseX                        = keys.addDoubleKey ("Body box base X"                           );
   public static final DoubleFootstepPlannerParameterKey  bodyBoxBaseY                        = keys.addDoubleKey ("Body box base Y"                           );
   public static final DoubleFootstepPlannerParameterKey  bodyBoxBaseZ                        = keys.addDoubleKey ("Body box base Z"                           );



   public static void main(String[] args)
   {
      FootstepPlannerParameterSet.printInitialSaveFileContents(keys.keys());

      FootstepPlannerParameterSet bipedParameters = new FootstepPlannerParameterSet(keys);

      LogTools.info("{}", bipedParameters.getValue(wiggleIntoConvexHullOfPlanarRegions));
      LogTools.info("{}", bipedParameters.getValue(rejectIfCannotFullyWiggleInside    ));
      LogTools.info("{}", bipedParameters.getValue(returnBestEffortPlan               ));
      LogTools.info("{}", bipedParameters.getValue(checkForBodyBoxCollisions          ));
      LogTools.info("{}", bipedParameters.getValue(performHeuristicSearchPolicies     ));
      LogTools.info("{}", bipedParameters.getValue(minimumStepsForBestEffortPlan      ));
      LogTools.info("{}", bipedParameters.getValue(idealFootstepWidth                 ));
      LogTools.info("{}", bipedParameters.getValue(idealFootstepLength                ));
      LogTools.info("{}", bipedParameters.getValue(wiggleInsideDelta                  ));
      LogTools.info("{}", bipedParameters.getValue(maximumXYWiggleDistance            ));
      LogTools.info("{}", bipedParameters.getValue(maximumYawWiggle                   ));
      LogTools.info("{}", bipedParameters.getValue(maxStepReach                       ));
      LogTools.info("{}", bipedParameters.getValue(maxStepYaw                         ));
      LogTools.info("{}", bipedParameters.getValue(minStepWidth                       ));
      LogTools.info("{}", bipedParameters.getValue(minStepLength                      ));
      LogTools.info("{}", bipedParameters.getValue(minStepYaw                         ));
      LogTools.info("{}", bipedParameters.getValue(maxStepZ                           ));
      LogTools.info("{}", bipedParameters.getValue(minFootholdPercent                 ));
      LogTools.info("{}", bipedParameters.getValue(minSurfaceIncline                  ));
      LogTools.info("{}", bipedParameters.getValue(maxStepWidth                       ));
      LogTools.info("{}", bipedParameters.getValue(minXClearanceFromStance            ));
      LogTools.info("{}", bipedParameters.getValue(minYClearanceFromStance            ));
      LogTools.info("{}", bipedParameters.getValue(maximumStepReachWhenSteppingUp     ));
      LogTools.info("{}", bipedParameters.getValue(maximumStepZWhenSteppingUp         ));
      LogTools.info("{}", bipedParameters.getValue(maximumStepXWhenForwardAndDown     ));
      LogTools.info("{}", bipedParameters.getValue(maximumStepZWhenForwardAndDown     ));
      LogTools.info("{}", bipedParameters.getValue(maximumZPenetrationOnValleyRegions ));
      LogTools.info("{}", bipedParameters.getValue(cliffHeightToAvoid                 ));
      LogTools.info("{}", bipedParameters.getValue(minimumDistanceFromCliffBottoms    ));
      LogTools.info("{}", bipedParameters.getValue(bodyGroundClearance                ));
      LogTools.info("{}", bipedParameters.getValue(bodyBoxWidth                       ));
      LogTools.info("{}", bipedParameters.getValue(bodyBoxHeight                      ));
      LogTools.info("{}", bipedParameters.getValue(bodyBoxDepth                       ));
      LogTools.info("{}", bipedParameters.getValue(bodyBoxBaseX                       ));
      LogTools.info("{}", bipedParameters.getValue(bodyBoxBaseY                       ));
      LogTools.info("{}", bipedParameters.getValue(bodyBoxBaseZ                       ));

      bipedParameters.setValue(BipedFootstepPlannerParameterKeys.maxStepReach, 0.35);

      bipedParameters.save();
   }
}
