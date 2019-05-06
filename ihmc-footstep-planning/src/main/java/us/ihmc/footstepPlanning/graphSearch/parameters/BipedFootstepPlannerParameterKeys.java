package us.ihmc.footstepPlanning.graphSearch.parameters;

import java.util.ArrayList;
import java.util.List;

public class BipedFootstepPlannerParameterKeys
{
   private static int idCount = 0;

   public static final List<FootstepPlannerParameterKey<?>> keys = new ArrayList<>();
   public static final String saveFileName = "bipedFootstepPlannerParameters";

   public static final FootstepPlannerParameterKey<Double > idealFootstepWidth                  = new FootstepPlannerParameterKey<>(keys, idCount++, "Ideal footstep width"                      );
   public static final FootstepPlannerParameterKey<Double > idealFootstepLength                 = new FootstepPlannerParameterKey<>(keys, idCount++, "Ideal footstep length"                     );
   public static final FootstepPlannerParameterKey<Double > wiggleInsideDelta                   = new FootstepPlannerParameterKey<>(keys, idCount++, "Wiggle inside delta"                       );
   public static final FootstepPlannerParameterKey<Boolean> wiggleIntoConvexHullOfPlanarRegions = new FootstepPlannerParameterKey<>(keys, idCount++, "Wiggle into convex hull of planar regions" );
   public static final FootstepPlannerParameterKey<Boolean> rejectIfCannotFullyWiggleInside     = new FootstepPlannerParameterKey<>(keys, idCount++, "Reject if cannot fully wiggle inside"      );
   public static final FootstepPlannerParameterKey<Double > maximumXYWiggleDistance             = new FootstepPlannerParameterKey<>(keys, idCount++, "Max XY wiggle distance"                    );
   public static final FootstepPlannerParameterKey<Double > maximumYawWiggle                    = new FootstepPlannerParameterKey<>(keys, idCount++, "Max yaw wiggle"                            );
   public static final FootstepPlannerParameterKey<Double > maxStepReach                        = new FootstepPlannerParameterKey<>(keys, idCount++, "Max step reach"                            );
   public static final FootstepPlannerParameterKey<Double > maxStepYaw                          = new FootstepPlannerParameterKey<>(keys, idCount++, "Max step yaw"                              );
   public static final FootstepPlannerParameterKey<Double > minStepWidth                        = new FootstepPlannerParameterKey<>(keys, idCount++, "Min step width"                            );
   public static final FootstepPlannerParameterKey<Double > minStepLength                       = new FootstepPlannerParameterKey<>(keys, idCount++, "Min step length"                           );
   public static final FootstepPlannerParameterKey<Double > minStepYaw                          = new FootstepPlannerParameterKey<>(keys, idCount++, "Min step yaw"                              );
   public static final FootstepPlannerParameterKey<Double > maxStepZ                            = new FootstepPlannerParameterKey<>(keys, idCount++, "Max step Z"                                );
   public static final FootstepPlannerParameterKey<Double > minFootholdPercent                  = new FootstepPlannerParameterKey<>(keys, idCount++, "Min foothold percent"                      );
   public static final FootstepPlannerParameterKey<Double > minSurfaceIncline                   = new FootstepPlannerParameterKey<>(keys, idCount++, "Min surface incline"                       );
   public static final FootstepPlannerParameterKey<Double > maxStepWidth                        = new FootstepPlannerParameterKey<>(keys, idCount++, "Max step width"                            );
   public static final FootstepPlannerParameterKey<Double > minXClearanceFromStance             = new FootstepPlannerParameterKey<>(keys, idCount++, "Min X clearance from stance"               );
   public static final FootstepPlannerParameterKey<Double > minYClearanceFromStance             = new FootstepPlannerParameterKey<>(keys, idCount++, "Min Y clearance from stance"               );
   public static final FootstepPlannerParameterKey<Double > maximumStepReachWhenSteppingUp      = new FootstepPlannerParameterKey<>(keys, idCount++, "Max step reach when stepping up"           );
   public static final FootstepPlannerParameterKey<Double > maximumStepZWhenSteppingUp          = new FootstepPlannerParameterKey<>(keys, idCount++, "Max step Z when stepping up"               );
   public static final FootstepPlannerParameterKey<Double > maximumStepXWhenForwardAndDown      = new FootstepPlannerParameterKey<>(keys, idCount++, "Max step X when forward and down"          );
   public static final FootstepPlannerParameterKey<Double > maximumStepZWhenForwardAndDown      = new FootstepPlannerParameterKey<>(keys, idCount++, "Max step Z when forward and down"          );
   public static final FootstepPlannerParameterKey<Double > maximumZPenetrationOnValleyRegions  = new FootstepPlannerParameterKey<>(keys, idCount++, "Max Z penetration on valley regions"       );
   public static final FootstepPlannerParameterKey<Double > cliffHeightToAvoid                  = new FootstepPlannerParameterKey<>(keys, idCount++, "Cliff height to avoid"                     );
   public static final FootstepPlannerParameterKey<Double > minimumDistanceFromCliffBottoms     = new FootstepPlannerParameterKey<>(keys, idCount++, "Min distance from cliff bottoms"           );
   public static final FootstepPlannerParameterKey<Boolean> returnBestEffortPlan                = new FootstepPlannerParameterKey<>(keys, idCount++, "Return best effort plan"                   );
   public static final FootstepPlannerParameterKey<Integer> minimumStepsForBestEffortPlan       = new FootstepPlannerParameterKey<>(keys, idCount++, "Min steps for best effort plan"            );
   public static final FootstepPlannerParameterKey<Double > bodyGroundClearance                 = new FootstepPlannerParameterKey<>(keys, idCount++, "Body ground clearance"                     );
   public static final FootstepPlannerParameterKey<Boolean> checkForBodyBoxCollisions           = new FootstepPlannerParameterKey<>(keys, idCount++, "Check for body box collisions"             );
   public static final FootstepPlannerParameterKey<Boolean> performHeuristicSearchPolicies      = new FootstepPlannerParameterKey<>(keys, idCount++, "Perform heuristic search policies"         );
   public static final FootstepPlannerParameterKey<Double > bodyBoxWidth                        = new FootstepPlannerParameterKey<>(keys, idCount++, "Body box width"                            );
   public static final FootstepPlannerParameterKey<Double > bodyBoxHeight                       = new FootstepPlannerParameterKey<>(keys, idCount++, "Body box height"                           );
   public static final FootstepPlannerParameterKey<Double > bodyBoxDepth                        = new FootstepPlannerParameterKey<>(keys, idCount++, "Body box depth"                            );
   public static final FootstepPlannerParameterKey<Double > bodyBoxBaseX                        = new FootstepPlannerParameterKey<>(keys, idCount++, "Body box base X"                           );
   public static final FootstepPlannerParameterKey<Double > bodyBoxBaseY                        = new FootstepPlannerParameterKey<>(keys, idCount++, "Body box base Y"                           );
   public static final FootstepPlannerParameterKey<Double > bodyBoxBaseZ                        = new FootstepPlannerParameterKey<>(keys, idCount++, "Body box base Z"                           );

   public static void main(String[] args)
   {
      FootstepPlannerParameterMap map = new FootstepPlannerParameterMap(BipedFootstepPlannerParameterKeys.keys, saveFileName);
      map.printInitializedSaveFile();

   }
}
