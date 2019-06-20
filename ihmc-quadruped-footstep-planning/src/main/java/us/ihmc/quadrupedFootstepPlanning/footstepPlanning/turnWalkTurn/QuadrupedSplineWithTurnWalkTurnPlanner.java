package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.turnWalkTurn;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsBasics;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.SplinePathPlanner;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedSplineWithTurnWalkTurnPlanner extends QuadrupedPathWithTurnWalkTurnPlanner
{
   private static final String prefix = "splineBased";

   public QuadrupedSplineWithTurnWalkTurnPlanner(QuadrupedXGaitSettingsBasics xGaitSettings, YoDouble timestamp, PointFootSnapperParameters pointFootSnapperParameters,
                                                 QuadrupedReferenceFrames referenceFrames, YoGraphicsListRegistry graphicsListRegistry,
                                                 YoVariableRegistry parentRegistry)
   {
      super(new SplinePathPlanner(prefix, parentRegistry), xGaitSettings, timestamp, pointFootSnapperParameters, referenceFrames, graphicsListRegistry, parentRegistry);
   }
}
