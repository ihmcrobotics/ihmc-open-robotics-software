package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.filters.AlphaFilteredYoFramePoint2D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLine2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CoPAndVelocityRotationEdgeCalculator implements RotationEdgeCalculator, SCS2YoGraphicHolder
{
   private final RotationEdgeCalculator velocityEdgeCalculator;

   private final AlphaFilteredYoFramePoint2D pointOfRotation;
   private final YoFrameVector2D axisOfRotation;
   private final FixedFrameLine2DBasics lineOfRotationInSole;

   private final EdgeVelocityStabilityEvaluator stabilityEvaluator;
   private final EdgeVisualizer edgeVisualizer;

   public CoPAndVelocityRotationEdgeCalculator(RobotSide side,
                                               MovingReferenceFrame soleFrame,
                                               YoPartialFootholdModuleParameters rotationParameters,
                                               double dt,
                                               YoRegistry parentRegistry,
                                               ColorDefinition color)
   {
      this(side,
           soleFrame,
           rotationParameters.getVelocityEdgeFilterBreakFrequency(),
           rotationParameters.getCopHistoryBreakFrequency(),
           rotationParameters.getStableRotationDirectionThreshold(),
           rotationParameters.getStableRotationPositionThreshold(),
           rotationParameters.getStableEdgeWindowSize(),
           dt,
           parentRegistry,
           color);
   }

   public CoPAndVelocityRotationEdgeCalculator(RobotSide side,
                                               MovingReferenceFrame soleFrame,
                                               DoubleProvider velocityEdgeFilterBreakFrequency,
                                               DoubleProvider copHistoryBreakFrequency,
                                               DoubleProvider stableRotationDirectionThreshold,
                                               DoubleProvider stableRotationPositionThreshold,
                                               IntegerProvider stableEdgeWindowSize,
                                               double dt,
                                               YoRegistry parentRegistry,
                                               ColorDefinition color)
   {
      this(side,
           soleFrame,
           new VelocityRotationEdgeCalculator(side,
                                              soleFrame,
                                              velocityEdgeFilterBreakFrequency,
                                              stableRotationDirectionThreshold,
                                              stableRotationPositionThreshold,
                                              stableEdgeWindowSize,
                                              dt,
                                              parentRegistry,
                                              null),
           copHistoryBreakFrequency,
           stableRotationDirectionThreshold,
           stableRotationPositionThreshold,
           stableEdgeWindowSize,
           dt,
           parentRegistry,
           color);
   }

   public CoPAndVelocityRotationEdgeCalculator(RobotSide side,
                                               ReferenceFrame soleFrame,
                                               RotationEdgeCalculator velocityEdgeCalculator,
                                               DoubleProvider copHistoryBreakFrequency,
                                               DoubleProvider stableRotationDirectionThreshold,
                                               DoubleProvider stableRotationPositionThreshold,
                                               IntegerProvider stableEdgeWindowSize,
                                               double dt,
                                               YoRegistry parentRegistry,
                                               ColorDefinition color)
   {
      this.velocityEdgeCalculator = velocityEdgeCalculator;

      String namePrefix = side.getLowerCaseName() + "CoPAndVelocity";
      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      DoubleProvider alpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(copHistoryBreakFrequency.getValue(), dt);
      pointOfRotation = new AlphaFilteredYoFramePoint2D(namePrefix + "PointOfRotation", "", registry, alpha, soleFrame);
      axisOfRotation = new YoFrameVector2D(namePrefix + "AxisOfRotation", soleFrame, registry);

      lineOfRotationInSole = new YoFrameLine2D(pointOfRotation, axisOfRotation);

      stabilityEvaluator = new EdgeVelocityStabilityEvaluator(namePrefix,
                                                              lineOfRotationInSole,
                                                              stableRotationDirectionThreshold,
                                                              stableRotationPositionThreshold,
                                                              stableEdgeWindowSize,
                                                              dt,
                                                              registry);

      edgeVisualizer = new EdgeVisualizer(namePrefix, color, registry);

      reset();

      parentRegistry.addChild(registry);
   }

   @Override
   public boolean compute(FramePoint2DReadOnly measuredCoP)
   {
      if (!measuredCoP.containsNaN())
         pointOfRotation.update(measuredCoP);

      if (pointOfRotation.containsNaN())
         pointOfRotation.set(measuredCoP);

      velocityEdgeCalculator.compute(measuredCoP);
      axisOfRotation.set(velocityEdgeCalculator.getLineOfRotation().getDirection());

      stabilityEvaluator.update();

      if (edgeVisualizer != null)
      {
         edgeVisualizer.visualize(true);
         edgeVisualizer.updateGraphics(lineOfRotationInSole);
      }

      return isRotationEdgeTrusted();
   }

   @Override
   public void reset()
   {
      velocityEdgeCalculator.reset();
      pointOfRotation.setToNaN();
      axisOfRotation.setToNaN();

      lineOfRotationInSole.setToNaN();

      stabilityEvaluator.reset();

      if (edgeVisualizer != null)
      {
         edgeVisualizer.visualize(false);
         edgeVisualizer.reset();
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
      return stabilityEvaluator.isEdgeVelocityStable();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(edgeVisualizer.getSCS2YoGraphics());

      return group;
   }
}
