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

      setCheckForBodyBoxCollisions(false);
      setIdealFootstepWidth(0.2);
      setIdealFootstepLength(0.2);
      setMaximumStepReach(0.4);
      setMaximumStepYaw(0.6);
      setMinimumStepYaw(-0.15);
      setMinimumStepWidth(0.2);
      setMaximumStepWidth(0.4);
      setMaximumStepZ(0.15);
      setBodyBoxBaseX(0.03);
      setBodyBoxBaseY(0.2);
      setBodyBoxBaseZ(0.3);
      setBodyBoxWidth(0.85);
      setBodyBoxDepth(0.4);
      setMinClearanceFromStance(0.05);
      setCliffBaseHeightToAvoid(0.07);
      setMinimumDistanceFromCliffBottoms(0.04);
      setWiggleInsideDelta(0.03);
      setMaximumXYWiggleDistance(0.04);
      setMaximumYawWiggle(0.3);
      setMaximumStepZWhenSteppingUp(0.05);
      setMaximumStepReachWhenSteppingUp(0.32);
      setMaximumStepZWhenForwardAndDown(0.05);
      setMaximumStepXWhenForwardAndDown(0.23);
      setAStarHeuristicsWeight(5.0);
      setYawWeight(0.15);
      setForwardWeight(2.5);
      setMaximum2dDistanceFromBoundingBoxToPenalize(0.05);

      load();
   }

   public static void main(String[] args)
   {
      ValkyrieFootstepPlannerParameters parameters = new ValkyrieFootstepPlannerParameters();
      parameters.save();
   }
}
