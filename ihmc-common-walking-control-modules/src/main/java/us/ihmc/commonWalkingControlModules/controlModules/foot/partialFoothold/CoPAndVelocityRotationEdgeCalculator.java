package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameLine2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;

import java.awt.*;

public class CoPAndVelocityRotationEdgeCalculator implements RotationEdgeCalculator
{
   private final RotationEdgeCalculator velocityEdgeCalculator;

   private final AlphaFilteredYoFramePoint2d pointOfRotation;
   private final YoFrameVector2D axisOfRotation;
   private final FixedFrameLine2DBasics lineOfRotationInSole;

   private final EdgeVelocityStabilityEvaluator stabilityEvaluator;
   private final EdgeVisualizer edgeVisualizer;

   public CoPAndVelocityRotationEdgeCalculator(RobotSide side,
                                               MovingReferenceFrame soleFrame,
                                               FootholdRotationParameters rotationParameters,
                                               double dt,
                                               YoVariableRegistry parentRegistry,
                                               YoGraphicsListRegistry graphicsListRegistry)
   {
      this(side,
           soleFrame,
           new VelocityRotationEdgeCalculator(side, soleFrame, rotationParameters, dt, parentRegistry, null),
           rotationParameters,
           dt,
           parentRegistry,
           graphicsListRegistry);
   }

   public CoPAndVelocityRotationEdgeCalculator(RobotSide side,
                                               ReferenceFrame soleFrame,
                                               RotationEdgeCalculator velocityEdgeCalculator,
                                               FootholdRotationParameters rotationParameters,
                                               double dt,
                                               YoVariableRegistry parentRegistry,
                                               YoGraphicsListRegistry graphicsListRegistry)
   {
      this.velocityEdgeCalculator = velocityEdgeCalculator;

      String namePrefix = side.getLowerCaseName() + "CoPAndVelocity";
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      DoubleProvider copAlpha = rotationParameters.getCopHistoryAlphaFilter();
      pointOfRotation = new AlphaFilteredYoFramePoint2d(namePrefix + "PointOfRotation", "", registry, copAlpha, soleFrame);
      axisOfRotation = new YoFrameVector2D(namePrefix + "AxisOfRotation", soleFrame, registry);

      lineOfRotationInSole = new YoFrameLine2D(pointOfRotation, axisOfRotation);

      stabilityEvaluator = new EdgeVelocityStabilityEvaluator(namePrefix,
                                                              lineOfRotationInSole,
                                                              rotationParameters.getStableRotationDirectionThreshold(),
                                                              rotationParameters.getStableRotationPositionThreshold(),
                                                              rotationParameters.getMinimumTicksForEstimate(),
                                                              dt,
                                                              registry);

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
      velocityEdgeCalculator.compute(measuredCoP);

      pointOfRotation.update(measuredCoP);
      axisOfRotation.set(velocityEdgeCalculator.getLineOfRotation().getDirection());

      stabilityEvaluator.update();

      if (edgeVisualizer != null)
      {
         edgeVisualizer.visualize(true);
         edgeVisualizer.updateGraphics(lineOfRotationInSole);
      }
   }

   @Override
   public void reset()
   {
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
      return stabilityEvaluator.isEdgeVelocityStable();
   }
}
