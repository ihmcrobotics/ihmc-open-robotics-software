package us.ihmc.quadrupedPlanning.footstepPlanning.turnWalkTurn;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedPlanning.pathPlanning.VisibilityGraphPathPlanner;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedVisGraphWithTurnWalkTurnPlanner extends QuadrupedPathWithTurnWalkTurnPlanner
{
   private static final String prefix = "visGraphBased";

   public QuadrupedVisGraphWithTurnWalkTurnPlanner(YoQuadrupedXGaitSettings xGaitSettings, VisibilityGraphsParameters visibilityGraphsParameters,
                                                   YoDouble timestamp, PointFootSnapperParameters pointFootSnapperParameters,
                                                   QuadrupedReferenceFrames referenceFrames, YoGraphicsListRegistry graphicsListRegistry,
                                                   YoVariableRegistry parentRegistry)
   {
      super(new VisibilityGraphPathPlanner(prefix, visibilityGraphsParameters, parentRegistry), xGaitSettings, timestamp, pointFootSnapperParameters,
            referenceFrames, graphicsListRegistry, parentRegistry);
   }
}
