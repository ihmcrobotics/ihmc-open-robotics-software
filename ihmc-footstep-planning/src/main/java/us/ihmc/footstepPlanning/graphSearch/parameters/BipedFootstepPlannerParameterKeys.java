package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.log.LogTools;
import us.ihmc.tools.property.*;

public class BipedFootstepPlannerParameterKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList("bipedFootstepPlannerParameters");

   public static final BooleanStoredPropertyKey wiggleIntoConvexHullOfPlanarRegions = keys.addBooleanKey("Wiggle into convex hull of planar regions" );
   public static final BooleanStoredPropertyKey rejectIfCannotFullyWiggleInside     = keys.addBooleanKey("Reject if cannot fully wiggle inside"      );
   public static final BooleanStoredPropertyKey returnBestEffortPlan                = keys.addBooleanKey("Return best effort plan"                   );
   public static final BooleanStoredPropertyKey checkForBodyBoxCollisions           = keys.addBooleanKey("Check for body box collisions"             );
   public static final BooleanStoredPropertyKey performHeuristicSearchPolicies      = keys.addBooleanKey("Perform heuristic search policies"         );
   public static final IntegerStoredPropertyKey minimumStepsForBestEffortPlan       = keys.addIntegerKey("Min steps for best effort plan"            );
   public static final DoubleStoredPropertyKey idealFootstepWidth                  = keys.addDoubleKey ("Ideal footstep width"                      );
   public static final DoubleStoredPropertyKey idealFootstepLength                 = keys.addDoubleKey ("Ideal footstep length"                     );
   public static final DoubleStoredPropertyKey wiggleInsideDelta                   = keys.addDoubleKey ("Wiggle inside delta"                       );
   public static final DoubleStoredPropertyKey maximumXYWiggleDistance             = keys.addDoubleKey ("Max XY wiggle distance"                    );
   public static final DoubleStoredPropertyKey maximumYawWiggle                    = keys.addDoubleKey ("Max yaw wiggle"                            );
   public static final DoubleStoredPropertyKey maxStepReach                        = keys.addDoubleKey ("Max step reach"                            );
   public static final DoubleStoredPropertyKey maxStepYaw                          = keys.addDoubleKey ("Max step yaw"                              );
   public static final DoubleStoredPropertyKey minStepWidth                        = keys.addDoubleKey ("Min step width"                            );
   public static final DoubleStoredPropertyKey minStepLength                       = keys.addDoubleKey ("Min step length"                           );
   public static final DoubleStoredPropertyKey minStepYaw                          = keys.addDoubleKey ("Min step yaw"                              );
   public static final DoubleStoredPropertyKey maxStepZ                            = keys.addDoubleKey ("Max step Z"                                );
   public static final DoubleStoredPropertyKey minFootholdPercent                  = keys.addDoubleKey ("Min foothold percent"                      );
   public static final DoubleStoredPropertyKey minSurfaceIncline                   = keys.addDoubleKey ("Min surface incline"                       );
   public static final DoubleStoredPropertyKey maxStepWidth                        = keys.addDoubleKey ("Max step width"                            );
   public static final DoubleStoredPropertyKey minXClearanceFromStance             = keys.addDoubleKey ("Min X clearance from stance"               );
   public static final DoubleStoredPropertyKey minYClearanceFromStance             = keys.addDoubleKey ("Min Y clearance from stance"               );
   public static final DoubleStoredPropertyKey maximumStepReachWhenSteppingUp      = keys.addDoubleKey ("Max step reach when stepping up"           );
   public static final DoubleStoredPropertyKey maximumStepZWhenSteppingUp          = keys.addDoubleKey ("Max step Z when stepping up"               );
   public static final DoubleStoredPropertyKey maximumStepXWhenForwardAndDown      = keys.addDoubleKey ("Max step X when forward and down"          );
   public static final DoubleStoredPropertyKey maximumStepZWhenForwardAndDown      = keys.addDoubleKey ("Max step Z when forward and down"          );
   public static final DoubleStoredPropertyKey maximumZPenetrationOnValleyRegions  = keys.addDoubleKey ("Max Z penetration on valley regions"       );
   public static final DoubleStoredPropertyKey cliffHeightToAvoid                  = keys.addDoubleKey ("Cliff height to avoid"                     );
   public static final DoubleStoredPropertyKey minimumDistanceFromCliffBottoms     = keys.addDoubleKey ("Min distance from cliff bottoms"           );
   public static final DoubleStoredPropertyKey bodyGroundClearance                 = keys.addDoubleKey ("Body ground clearance"                     );
   public static final DoubleStoredPropertyKey bodyBoxWidth                        = keys.addDoubleKey ("Body box width"                            );
   public static final DoubleStoredPropertyKey bodyBoxHeight                       = keys.addDoubleKey ("Body box height"                           );
   public static final DoubleStoredPropertyKey bodyBoxDepth                        = keys.addDoubleKey ("Body box depth"                            );
   public static final DoubleStoredPropertyKey bodyBoxBaseX                        = keys.addDoubleKey ("Body box base X"                           );
   public static final DoubleStoredPropertyKey bodyBoxBaseY                        = keys.addDoubleKey ("Body box base Y"                           );
   public static final DoubleStoredPropertyKey bodyBoxBaseZ                        = keys.addDoubleKey ("Body box base Z"                           );



   public static void main(String[] args)
   {
      StoredPropertySet.printInitialSaveFileContents(keys.keys());

      StoredPropertySet bipedParameters = new StoredPropertySet(keys,
                                                                BipedFootstepPlannerParameterKeys.class,
                                                                "ihmc-open-robotics-software",
                                                                "ihmc-footstep-planning/src/main/resources/");

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
