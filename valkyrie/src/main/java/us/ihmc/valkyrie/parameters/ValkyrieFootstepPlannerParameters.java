package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.*;
import us.ihmc.tools.property.StoredPropertySet;

public class ValkyrieFootstepPlannerParameters extends StoredPropertySet implements FootstepPlannerParametersBasics
{
   public ValkyrieFootstepPlannerParameters()
   {
      this("ihmc-open-robotics-software", "valkyrie/src/main/resources");
   }

   public ValkyrieFootstepPlannerParameters(String projectName, String pathToResources)
   {
      super(FootstepPlannerParameterKeys.keys, ValkyrieFootstepPlannerParameters.class, projectName, pathToResources);

      setCheckForBodyBoxCollisions(true);
      setIdealFootstepWidth(0.2);
      setIdealFootstepLength(0.2);
      setMaximumStepReach(0.4);
      setMaximumStepYaw(0.6);
      setMinimumStepYaw(-0.15);
      setMinimumStepWidth(0.2);
      setMaximumStepWidth(0.4);
      setMaximumStepZ(0.15);
      setReturnBestEffortPlan(false);
      setBodyBoxBaseX(0.03);
      setBodyBoxBaseY(0.2);
      setBodyBoxBaseZ(0.1);
      setBodyBoxWidth(0.85);
      setBodyBoxDepth(0.4);
      setMinimumStepsForBestEffortPlan(3);
      setMinXClearanceFromStance(0.2);
      setMinYClearanceFromStance(0.2);
      setCliffHeightToAvoid(0.07);
      setMinimumDistanceFromCliffBottoms(0.04);
      setWiggleInsideDelta(0.03);
      setWiggleIntoConvexHullOfPlanarRegions(true);
      setMaximumXYWiggleDistance(0.04);
      setMaximumYawWiggle(0.3);
      setRejectIfCannotFullyWiggleInside(false);
      setMaximumStepZWhenSteppingUp(0.05);
      setMaximumStepReachWhenSteppingUp(0.32);
      setMaximumStepZWhenForwardAndDown(0.05);
      setMaximumStepXWhenForwardAndDown(0.23);
      setAStarHeuristicsWeight(5.0);
      setYawWeight(0.15);
      setUseQuadraticDistanceCost(true);
      setForwardWeight(2.5);
      setMaximum2dDistanceFromBoundingBoxToPenalize(0.05);
      setBoundingBoxCost(0.0);

      load();
   }

   public static void main(String[] args)
   {
      ValkyrieFootstepPlannerParameters parameters = new ValkyrieFootstepPlannerParameters();
      parameters.save();
   }
}
