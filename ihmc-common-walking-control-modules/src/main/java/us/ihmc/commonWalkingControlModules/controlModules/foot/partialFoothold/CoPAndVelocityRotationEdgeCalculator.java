package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameLine2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;

import java.awt.*;

public class CoPAndVelocityRotationEdgeCalculator implements RotationEdgeCalculator
{
   private final RotationEdgeCalculator copHistoryEdgeCalculator;
   private final RotationEdgeCalculator velocityEdgeCalculator;

   private final YoFramePoint2D pointOfRotation;
   private final YoFrameVector2D axisOfRotation;
   private final FixedFrameLine2DBasics lineOfRotationInSole;

   private final EdgeVelocityStabilityEvaluator stabilityEvaluator;
   private final EdgeVisualizer edgeVisualizer;

   public CoPAndVelocityRotationEdgeCalculator(RobotSide side, MovingReferenceFrame soleFrame, ExplorationParameters explorationParameters, double dt,
                                               YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this(side, soleFrame, new CoPHistoryRotationEdgeCalculator(side, soleFrame, explorationParameters, dt, parentRegistry, null),
           new VelocityRotationEdgeCalculator(side, soleFrame, explorationParameters, dt, parentRegistry, null), explorationParameters, dt, parentRegistry,
           graphicsListRegistry);
   }

   public CoPAndVelocityRotationEdgeCalculator(RobotSide side, ReferenceFrame soleFrame, RotationEdgeCalculator copHistoryEdgeCalculator,
                                               RotationEdgeCalculator velocityEdgeCalculator, ExplorationParameters explorationParameters, double dt,
                                               YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.copHistoryEdgeCalculator = copHistoryEdgeCalculator;
      this.velocityEdgeCalculator = velocityEdgeCalculator;

      String namePrefix = side.getLowerCaseName() + "CoPAndVelocity";
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      pointOfRotation = new YoFramePoint2D(namePrefix + "PointOfRotation", soleFrame, registry);
      axisOfRotation = new YoFrameVector2D(namePrefix + "AxisOfRotation", soleFrame, registry);

      lineOfRotationInSole = new YoFrameLine2D(pointOfRotation, axisOfRotation);

      stabilityEvaluator = new EdgeVelocityStabilityEvaluator(namePrefix, lineOfRotationInSole, explorationParameters.getStableLoRAngularVelocityThreshold(),
                                                              explorationParameters.getStableCoRLinearVelocityThreshold(), dt, registry);

      if (graphicsListRegistry != null)
         edgeVisualizer = new EdgeVisualizer(namePrefix, Color.RED, registry, graphicsListRegistry);
      else
         edgeVisualizer = null;

      reset();

      parentRegistry.addChild(registry);
   }

   @Override
   public void compute(FramePoint2DReadOnly measuredCoP)
   {
      copHistoryEdgeCalculator.compute(measuredCoP);
      velocityEdgeCalculator.compute(measuredCoP);

      pointOfRotation.set(copHistoryEdgeCalculator.getLineOfRotation().getPoint());
      axisOfRotation.set(velocityEdgeCalculator.getLineOfRotation().getDirection());

      stabilityEvaluator.update();

      if (edgeVisualizer != null)
      {
         edgeVisualizer.visualize(stabilityEvaluator.isEdgeVelocityStable());
         edgeVisualizer.updateGraphics(lineOfRotationInSole);
      }
   }

   @Override
   public void reset()
   {
      copHistoryEdgeCalculator.reset();
      velocityEdgeCalculator.reset();

      lineOfRotationInSole.setToNaN();

      stabilityEvaluator.reset();

      if (edgeVisualizer != null)
         edgeVisualizer.reset();
   }

   @Override
   public FrameLine2DReadOnly getLineOfRotation()
   {
      return lineOfRotationInSole;
   }

   @Override
   public boolean isRotationEdgeTrusted()
   {
      if (!copHistoryEdgeCalculator.isRotationEdgeTrusted() || !velocityEdgeCalculator.isRotationEdgeTrusted())
         return false;

      return stabilityEvaluator.isEdgeVelocityStable();
   }
}
