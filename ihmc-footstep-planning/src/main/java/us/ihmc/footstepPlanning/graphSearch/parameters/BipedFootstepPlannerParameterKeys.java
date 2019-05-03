package us.ihmc.footstepPlanning.graphSearch.parameters;

import java.util.ArrayList;
import java.util.List;

public class BipedFootstepPlannerParameterKeys implements FootstepPlannerParameterKeys
{
   private static int idCount = 0;

   public static final List<FootstepPlannerParameterKey<?>> keys = new ArrayList<>();
   public static final String saveFileName = "bipedFootstepPlannerParameters.ini";

   public static final FootstepPlannerParameterKey<Double > idealFootstepWidth                  = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > idealFootstepLength                 = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > wiggleInsideDelta                   = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Boolean> wiggleIntoConvexHullOfPlanarRegions = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Boolean> rejectIfCannotFullyWiggleInside     = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > maximumXYWiggleDistance             = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > maximumYawWiggle                    = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > maxStepReach                        = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > maxStepYaw                          = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > minStepWidth                        = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > minStepLength                       = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > minStepYaw                          = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > maxStepZ                            = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > minFootholdPercent                  = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > minSurfaceIncline                   = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > maxStepWidth                        = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > minXClearanceFromStance             = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > minYClearanceFromStance             = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > maximumStepReachWhenSteppingUp      = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > maximumStepZWhenSteppingUp          = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > maximumStepXWhenForwardAndDown      = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > maximumStepZWhenForwardAndDown      = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > maximumZPenetrationOnValleyRegions  = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > cliffHeightToAvoid                  = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > minimumDistanceFromCliffBottoms     = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Boolean> returnBestEffortPlan                = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Integer> minimumStepsForBestEffortPlan       = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > bodyGroundClearance                 = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Boolean> checkForBodyBoxCollisions           = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Boolean> performHeuristicSearchPolicies      = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > bodyBoxWidth                        = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > bodyBoxHeight                       = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > bodyBoxDepth                        = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > bodyBoxBaseX                        = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > bodyBoxBaseY                        = new FootstepPlannerParameterKey<>(keys, idCount++, "");
   public static final FootstepPlannerParameterKey<Double > bodyBoxBaseZ                        = new FootstepPlannerParameterKey<>(keys, idCount++, "");

   @Override
   public List<FootstepPlannerParameterKey<?>> getKeys()
   {
      return keys;
   }

   @Override
   public String getSaveFileName()
   {
      return saveFileName;
   }
}
