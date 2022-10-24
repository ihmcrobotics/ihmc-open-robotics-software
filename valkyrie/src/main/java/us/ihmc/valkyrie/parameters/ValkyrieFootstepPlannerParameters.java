package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.*;
import us.ihmc.tools.property.StoredPropertySet;

public class ValkyrieFootstepPlannerParameters extends StoredPropertySet implements FootstepPlannerParametersBasics
{
   public static final String PROJECT_NAME = "ihmc-open-robotics-software";
   public static final String PATH_TO_RESOURCES = "valkyrie/src/main/resources";

   public ValkyrieFootstepPlannerParameters()
   {
      this("");
   }

   public ValkyrieFootstepPlannerParameters(String fileNameSuffix)
   {
      this(PROJECT_NAME, PATH_TO_RESOURCES, fileNameSuffix);
   }

   public ValkyrieFootstepPlannerParameters(String projectName, String pathToResources)
   {
      this(projectName, pathToResources, "");
   }

   public ValkyrieFootstepPlannerParameters(String projectName, String pathToResources, String fileNameSuffix)
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
      setWiggleInsideDeltaTarget(0.03);
      setMaximumXYWiggleDistance(0.04);
      setMaximumYawWiggle(0.3);
      setMaximumStepZWhenSteppingUp(0.05);
      setMaximumStepReachWhenSteppingUp(0.32);
      setMaximumStepZWhenForwardAndDown(0.05);
      setMaximumStepXWhenForwardAndDown(0.23);
      setAStarHeuristicsWeight(5.0);
      setYawWeight(0.15);
      setForwardWeight(2.5);

      loadUnsafe();
   }

   public static void main(String[] args)
   {
      ValkyrieFootstepPlannerParameters parameters = new ValkyrieFootstepPlannerParameters();
      parameters.save();
   }
}
