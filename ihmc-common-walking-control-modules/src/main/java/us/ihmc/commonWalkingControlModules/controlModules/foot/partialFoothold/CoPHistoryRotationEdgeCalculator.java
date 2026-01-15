package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.functionApproximation.OnlineLine2DLinearRegression;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLine2D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * This class aims to calculate the edge of rotation of the foot by fitting a line to the the CoP history when the foot is rotating.
 * It likely will not produce a good estimate of the actual direction of the rotation, because the measured CoP is likely going to be similar
 * to the same location, unless the foot rotates for a while. However, it will provide a good estimate of the location of the line.
 */
public class CoPHistoryRotationEdgeCalculator implements RotationEdgeCalculator, SCS2YoGraphicHolder
{
   private final OnlineLine2DLinearRegression lineCalculator;
   private final YoFrameLine2D lineOfRotationInSole;

   private final EdgeVelocityStabilityEvaluator stabilityEvaluator;
   private final EdgeVisualizer edgeVisualizer;
   private final DoubleProvider inlineStdDevThreshold;
   private final DoubleProvider transverseStdDevThreshold;
   private final YoBoolean statisticsStable;
   private final YoBoolean lineStable;

   public CoPHistoryRotationEdgeCalculator(RobotSide side,
                                           ReferenceFrame soleFrame,
                                           YoPartialFootholdModuleParameters rotationParameters,
                                           double dt,
                                           YoRegistry parentRegistry,
                                           ColorDefinition color)
   {
      this(side,
           soleFrame,
           rotationParameters.getStableRotationDirectionThreshold(),
           rotationParameters.getStableRotationPositionThreshold(),
           rotationParameters.getStableEdgeWindowSize(),
           rotationParameters.getInlineCoPHistoryStdDev(),
           rotationParameters.getTransverseCoPHistoryStdDev(),
           dt,
           parentRegistry,
           color);
   }

   public CoPHistoryRotationEdgeCalculator(RobotSide side,
                                           ReferenceFrame soleFrame,
                                           DoubleProvider stableRotationDirectionThreshold,
                                           DoubleProvider stableRotationPositionThreshold,
                                           IntegerProvider stableEdgeWindowSize,
                                           DoubleProvider inlineStdDevThreshold,
                                           DoubleProvider transverseStdDevThreshold,
                                           double dt,
                                           YoRegistry parentRegistry,
                                           ColorDefinition color)
   {
      String namePrefix = side.getLowerCaseName() + "CoPHistory";
      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      lineStable = new YoBoolean(namePrefix + "IsStable", registry);
      statisticsStable = new YoBoolean(namePrefix + "StatisticsStable", registry);

      this.inlineStdDevThreshold = inlineStdDevThreshold;
      this.transverseStdDevThreshold = transverseStdDevThreshold;

      lineCalculator = new OnlineLine2DLinearRegression(namePrefix + "FootRotation", registry);
      lineOfRotationInSole = new YoFrameLine2D(namePrefix + "LineOfRotation", "", soleFrame, registry);

      stabilityEvaluator = new EdgeVelocityStabilityEvaluator(namePrefix,
                                                              lineOfRotationInSole,
                                                              stableRotationDirectionThreshold,
                                                              stableRotationPositionThreshold,
                                                              stableEdgeWindowSize,
                                                              dt,
                                                              registry);

      edgeVisualizer = new EdgeVisualizer(namePrefix, color, registry);

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
   public boolean compute(FramePoint2DReadOnly measuredCoP)
   {
      lineCalculator.update(measuredCoP);
      lineOfRotationInSole.set(lineCalculator.getMeanLine());

      stabilityEvaluator.update();

      statisticsStable.set(lineCalculator.getTransverseStandardDeviation() < transverseStdDevThreshold.getValue()
                       && lineCalculator.getInlineStandardDeviation() > inlineStdDevThreshold.getValue());
      lineStable.set(statisticsStable.getBooleanValue() && stabilityEvaluator.isEdgeVelocityStable());

      if (edgeVisualizer != null)
      {
         edgeVisualizer.visualize(true);
         edgeVisualizer.updateGraphics(lineOfRotationInSole);
      }

      return isRotationEdgeTrusted();
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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(edgeVisualizer.getSCS2YoGraphics());

      return group;
   }
}
