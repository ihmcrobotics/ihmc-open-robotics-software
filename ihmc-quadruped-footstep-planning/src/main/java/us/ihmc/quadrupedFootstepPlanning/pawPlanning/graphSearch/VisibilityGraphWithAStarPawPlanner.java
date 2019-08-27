package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.VisibilityGraphPawPathPlanner;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class VisibilityGraphWithAStarPawPlanner extends BodyPathAndPawPlannerWrapper
{
   private static final String prefix = "VisGraph";

   public VisibilityGraphWithAStarPawPlanner(PawStepPlannerParametersReadOnly parameters, QuadrupedXGaitSettingsReadOnly xGaitSettingsReadOnly,
                                             VisibilityGraphsParametersReadOnly visibilityGraphsParameters, YoGraphicsListRegistry graphicsListRegistry,
                                             YoVariableRegistry parentRegistry)
   {
      super(prefix, parameters, parentRegistry, graphicsListRegistry);

      waypointPathPlanner = new VisibilityGraphPawPathPlanner(prefix, visibilityGraphsParameters, parentRegistry);
      pawStepPlanner = new BodyPathBasedAStarPawPlanner(prefix, bodyPathPlanner, parameters, xGaitSettingsReadOnly, registry);
   }
}
