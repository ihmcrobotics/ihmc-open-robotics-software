package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.VisibilityGraphPawPathPlanner;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class VisibilityGraphWithAStarPawPlanner extends BodyPathAndPawPlannerWrapper
{
   private static final String prefix = "VisGraph";

   public VisibilityGraphWithAStarPawPlanner(PawPlannerParameters parameters, QuadrupedXGaitSettingsReadOnly xGaitSettingsReadOnly,
                                             VisibilityGraphsParameters visibilityGraphsParameters, YoGraphicsListRegistry graphicsListRegistry,
                                             YoVariableRegistry parentRegistry)
   {
      super(prefix, parameters, parentRegistry, graphicsListRegistry);

      waypointPathPlanner = new VisibilityGraphPawPathPlanner(prefix, visibilityGraphsParameters, parentRegistry);
      pawPlanner = new BodyPathBasedAStarPawPlanner(prefix, bodyPathPlanner, parameters, xGaitSettingsReadOnly, registry);
   }
}
