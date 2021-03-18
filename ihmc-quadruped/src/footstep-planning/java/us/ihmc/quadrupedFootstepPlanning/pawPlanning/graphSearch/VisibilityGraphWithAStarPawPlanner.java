package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAndCliffAvoidanceProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.VisibilityGraphPawPathPlanner;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

public class VisibilityGraphWithAStarPawPlanner extends BodyPathAndPawPlannerWrapper
{
   private static final String prefix = "VisGraph";

   public VisibilityGraphWithAStarPawPlanner(PawStepPlannerParametersReadOnly parameters, QuadrupedXGaitSettingsReadOnly xGaitSettingsReadOnly,
                                             VisibilityGraphsParametersReadOnly visibilityGraphsParameters, YoGraphicsListRegistry graphicsListRegistry,
                                             YoRegistry parentRegistry)
   {
      super(prefix, parentRegistry, graphicsListRegistry);

      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(visibilityGraphsParameters);
      waypointPathPlanner = new VisibilityGraphPawPathPlanner(prefix, visibilityGraphsParameters, postProcessor, parentRegistry);
      pawStepPlanner = new BodyPathBasedAStarPawPlanner(prefix, bodyPathPlanner, parameters, xGaitSettingsReadOnly, registry);
   }
}
