package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class AtlasFootstepPlannerParameters extends StoredPropertySet implements FootstepPlannerParametersBasics
{
   public AtlasFootstepPlannerParameters()
   {
      this("ihmc-open-robotics-software", "atlas/src/main/resources");
   }

   public AtlasFootstepPlannerParameters(String projectName, String pathToResources)
   {
      super(FootstepPlannerParameterKeys.keys, AtlasFootstepPlannerParameters.class, projectName, pathToResources);

      setWiggleIntoConvexHullOfPlanarRegions(true);
      setRejectIfCannotFullyWiggleInside(false);
      setReturnBestEffortPlan(false);
      setCheckForBodyBoxCollisions(false);
      setCheckForPathCollisions(true);
      setPerformHeuristicSearchPolicies(true);
      setMinimumStepsForBestEffortPlan(3);
      setCliffHeightToAvoid(0.05);
      setMinimumDistanceFromCliffBottoms(0.1);
      setMaximumStepReach(0.5);
      setMinimumStepLength(-0.6);
      setMaximumStepWidth(0.4);
      setMinimumStepWidth(0.15);
      setMaximumStepYaw(0.5);
      setMinimumStepYaw(-0.5);
      setMaximumStepZ(0.25);
      setMaximumXYWiggleDistance(0.1);
      setMaximumYawWiggle(0.09);
      setMinimumFootholdPercent(0.99);
      setMinimumSurfaceInclineRadians(0.78539);
      setMinXClearanceFromStance(0.22);
      setMinYClearanceFromStance(0.22);
      setWiggleInsideDelta(0.02);
      setMaximumStepZWhenSteppingUp(1.5);
      setMaximumStepReachWhenSteppingUp(0.5);
      setMaximumStepZWhenForwardAndDown(1.0);
      setMaximumStepXWhenForwardAndDown(1.5);
      setMaximumZPenetrationOnValleyRegions(Double.POSITIVE_INFINITY);
      setIdealFootstepWidth(0.22);
      setIdealFootstepLength(0.3);
      setBodyGroundClearance(0.25);
      setBodyBoxWidth(0.7);
      setBodyBoxHeight(1.5);
      setBodyBoxDepth(0.3);
      setBodyBoxBaseX(0.0);
      setBodyBoxBaseY(0.0);
      setBodyBoxBaseZ(0.25);
      setFinalTurnProximity(1.0);

      load();
   }

   public static void main(String[] args)
   {
      AtlasFootstepPlannerParameters parameters = new AtlasFootstepPlannerParameters();
      parameters.save();
   }
}