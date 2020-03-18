package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.functionApproximation.OnlineLine2DLinearRegression;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFrameLine2D;

import java.awt.*;

/**
 * This class aims to calculate the edge of rotation of the foot by fitting a line to the the CoP history when the foot is rotating.
 * It likely will not produce a good estimate of the actual direction of the rotation, because the measured CoP is likely going to be similar
 * to the same location, unless the foot rotates for a while. However, it will provide a good estimate of the location of the line.
 */
public class CoPHistoryRotationEdgeCalculator implements RotationEdgeCalculator
{
   private final OnlineLine2DLinearRegression lineCalculator;
   private final YoFrameLine2D lineOfRotationInSole;

   private final EdgeVelocityStabilityEvaluator stabilityEvaluator;
   private final EdgeVisualizer edgeVisualizer;
   private final DoubleProvider lineStdDevThreshold = () -> 0.01;
   private final DoubleProvider transverseStdDevThreshold = () -> 0.002;
   private final YoBoolean lineStable;

   public CoPHistoryRotationEdgeCalculator(RobotSide side,
                                           ReferenceFrame soleFrame,
                                           FootholdRotationParameters rotationParameters,
                                           double dt,
                                           YoVariableRegistry parentRegistry,
                                           YoGraphicsListRegistry graphicsListRegistry)
   {
      this(side,
           soleFrame,
           rotationParameters.getStableRotationDirectionThreshold(),
           rotationParameters.getStableRotationPositionThreshold(),
           rotationParameters.getStableEdgeWindowSize(),
           dt,
           parentRegistry,
           graphicsListRegistry);
   }

   public CoPHistoryRotationEdgeCalculator(RobotSide side,
                                           ReferenceFrame soleFrame,
                                           DoubleProvider stableRotationDirectionThreshold,
                                           DoubleProvider stableRotationPositionThreshold,
                                           IntegerProvider stableEdgeWindowSize,
                                           double dt,
                                           YoVariableRegistry parentRegistry,
                                           YoGraphicsListRegistry graphicsListRegistry)
   {
      String namePrefix = side.getLowerCaseName() + "CoPHistory";
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      lineStable = new YoBoolean(namePrefix + "IsStable", registry);

      lineCalculator = new OnlineLine2DLinearRegression(namePrefix + "FootRotation", registry);
      lineOfRotationInSole = new YoFrameLine2D(namePrefix + "LineOfRotation", "", soleFrame, registry);

      stabilityEvaluator = new EdgeVelocityStabilityEvaluator(namePrefix,
                                                              lineOfRotationInSole,
                                                              stableRotationDirectionThreshold,
                                                              stableRotationPositionThreshold,
                                                              stableEdgeWindowSize,
                                                              dt,
                                                              registry);

      if (graphicsListRegistry != null)
         edgeVisualizer = new EdgeVisualizer(namePrefix, Color.BLUE, registry, graphicsListRegistry);
      else
         edgeVisualizer = null;

      parentRegistry.addChild(registry);
   }

   @Override
   public void reset()
   {
      if (edgeVisualizer != null)
         edgeVisualizer.reset();

      stabilityEvaluator.reset();
      lineCalculator.reset();
   }

   @Override
   public void compute(FramePoint2DReadOnly measuredCoP)
   {
      lineCalculator.update(measuredCoP);
      lineOfRotationInSole.set(lineCalculator.getMeanLine());

      boolean stable = lineCalculator.getTransverseStandardDeviation() < transverseStdDevThreshold.getValue()
                       && lineCalculator.getInlineStandardDeviation() > lineStdDevThreshold.getValue();
      lineStable.set(stable && stabilityEvaluator.isEdgeVelocityStable());

      if (edgeVisualizer != null)
      {
         edgeVisualizer.visualize(true);
         edgeVisualizer.updateGraphics(lineOfRotationInSole);
      }
   }

   @Override
   public FrameLine2DReadOnly getLineOfRotation()
   {
      return lineOfRotationInSole;
   }

   @Override
   public boolean isRotationEdgeTrusted()
   {
      return lineStable.getBooleanValue();
   }
}
