package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.statistics.Line2DStatisticsCalculator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLine2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class VelocityRotationEdgeCalculator implements RotationEdgeCalculator
{
   private final MovingReferenceFrame soleFrame;

   private final FixedFramePoint2DBasics pointOfRotation;
   private final AlphaFilteredYoFramePoint2d filteredPointOfRotation;

   private final FixedFrameVector2DBasics axisOfRotation;
   private final AlphaFilteredYoFrameVector2d filteredAxisOfRotation;

   private final FixedFrameLine2DBasics lineOfRotationInSole;

   private final Line2DStatisticsCalculator lineOfRotationStandardDeviation;

   private final EdgeVisualizer edgeVisualizer;

   private final EdgeVelocityStabilityEvaluator stabilityEvaluator;

   public VelocityRotationEdgeCalculator(RobotSide side,
                                         MovingReferenceFrame soleFrame,
                                         FootholdRotationParameters rotationParameters,
                                         double dt,
                                         YoRegistry parentRegistry,
                                         YoGraphicsListRegistry graphicsListRegistry)
   {
      this(side,
           soleFrame,
           rotationParameters.getVelocityEdgeFilterBreakFrequency(),
           rotationParameters.getStableRotationDirectionThreshold(),
           rotationParameters.getStableRotationPositionThreshold(),
           rotationParameters.getStableEdgeWindowSize(),
           dt,
           parentRegistry,
           graphicsListRegistry);

   }
   public VelocityRotationEdgeCalculator(RobotSide side,
                                         MovingReferenceFrame soleFrame,
                                         DoubleProvider velocityEdgeFilterBreakFrequency,
                                         DoubleProvider stableRotationDirectionThreshold,
                                         DoubleProvider stableRotationPositionThreshold,
                                         IntegerProvider stableEdgeWindowSize,
                                         double dt,
                                         YoRegistry parentRegistry,
                                         YoGraphicsListRegistry graphicsListRegistry)
   {
      this.soleFrame = soleFrame;

      String namePrefix = side.getLowerCaseName() + "Velocity";
      YoRegistry registry = new YoRegistry(getClass().getSimpleName() + side.getPascalCaseName());

      pointOfRotation = new YoFramePoint2D(namePrefix + "PointOfRotation", soleFrame, registry);
      axisOfRotation = new YoFrameVector2D(namePrefix + "AxisOfRotation", soleFrame, registry);
      parentRegistry.addChild(registry);

      DoubleProvider alpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(velocityEdgeFilterBreakFrequency.getValue(), dt);
      filteredPointOfRotation = new AlphaFilteredYoFramePoint2d(namePrefix + "FilteredPointOfRotation", "", registry, alpha, pointOfRotation);
      filteredAxisOfRotation = new AlphaFilteredYoFrameVector2d(namePrefix + "FilteredAxisOfRotation", "", registry, alpha, axisOfRotation);

      lineOfRotationInSole = new YoFrameLine2D(filteredPointOfRotation, filteredAxisOfRotation);

      lineOfRotationStandardDeviation = new Line2DStatisticsCalculator(namePrefix + "LineOfRotation", lineOfRotationInSole, registry);

      if (graphicsListRegistry != null)
         edgeVisualizer = new EdgeVisualizer(namePrefix, Color.GREEN, registry, graphicsListRegistry);
      else
         edgeVisualizer = null;

      stabilityEvaluator = new EdgeVelocityStabilityEvaluator(namePrefix,
                                                              lineOfRotationInSole,
                                                              stableRotationDirectionThreshold,
                                                              stableRotationPositionThreshold,
                                                              stableEdgeWindowSize,
                                                              dt,
                                                              registry);

      reset();
   }

   private final FrameVector3D tempPointOfRotation = new FrameVector3D();

   @Override
   public void reset()
   {
      if (edgeVisualizer != null)
         edgeVisualizer.reset();

      filteredPointOfRotation.reset();
      filteredAxisOfRotation.reset();
      lineOfRotationInSole.setToZero();

      stabilityEvaluator.reset();

      lineOfRotationStandardDeviation.reset();
   }

   @Override
   public boolean compute(FramePoint2DReadOnly measuredCoP)
   {
      TwistReadOnly soleFrameTwist = soleFrame.getTwistOfFrame();

      double omegaSquared = soleFrameTwist.getAngularPart().lengthSquared();
      double omega = EuclidCoreTools.fastSquareRoot(omegaSquared);

      tempPointOfRotation.setToZero(soleFrame);
      tempPointOfRotation.cross(soleFrameTwist.getAngularPart(), soleFrameTwist.getLinearPart());
      tempPointOfRotation.scale(1.0 / omegaSquared);
      pointOfRotation.set(tempPointOfRotation);

      axisOfRotation.set(soleFrameTwist.getAngularPart());
      axisOfRotation.scale(1.0 / omega);

      if (axisOfRotation.dot(filteredAxisOfRotation) < 0.0)
      {
         axisOfRotation.negate();
      }

      // Filter the line of rotation:
      filteredPointOfRotation.update();
      filteredAxisOfRotation.update();

      lineOfRotationStandardDeviation.update();

      stabilityEvaluator.update();

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
      return stabilityEvaluator.isEdgeVelocityStable();
   }
}
