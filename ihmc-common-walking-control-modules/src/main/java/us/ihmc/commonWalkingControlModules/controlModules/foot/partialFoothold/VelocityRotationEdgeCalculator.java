package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
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
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameLine2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;

import java.awt.*;

public class VelocityRotationEdgeCalculator implements RotationEdgeCalculator
{
   private final MovingReferenceFrame soleFrame;

   private final FixedFramePoint2DBasics pointOfRotation;
   private final AlphaFilteredYoFramePoint2d filteredPointOfRotation;

   private final FixedFrameVector2DBasics axisOfRotation;
   private final AlphaFilteredYoFrameVector2d filteredAxisOfRotation;

   private final FixedFrameLine2DBasics lineOfRotationInSole;

   private final DoubleProvider filterBreakFrequency;

   private final Line2DStatisticsCalculator lineOfRotationStandardDeviation;

   private final EdgeVisualizer edgeVisualizer;

   private final EdgeVelocityStabilityEvaluator stabilityEvaluator;

   public VelocityRotationEdgeCalculator(RobotSide side, MovingReferenceFrame soleFrame, ExplorationParameters explorationParameters, double dt,
                                         YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.soleFrame = soleFrame;

      String namePrefix = side.getLowerCaseName() + "Velocity";
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());

      pointOfRotation = new YoFramePoint2D(namePrefix + "PointOfRotation", soleFrame, registry);
      axisOfRotation = new YoFrameVector2D(namePrefix + "AxisOfRotation", soleFrame, registry);

      String feetManagerName = FeetManager.class.getSimpleName();
      String paramRegistryName = getClass().getSimpleName() + "Parameters";
      filterBreakFrequency = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "filterBreakFrequency", registry, 1.0);

      DoubleProvider alpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filterBreakFrequency.getValue(), dt);
      filteredPointOfRotation = new AlphaFilteredYoFramePoint2d(namePrefix + "FilteredPointOfRotation", "", registry, alpha, pointOfRotation);
      filteredAxisOfRotation = new AlphaFilteredYoFrameVector2d(namePrefix + "FilteredAxisOfRotation", "", registry, alpha, axisOfRotation);

      lineOfRotationInSole = new YoFrameLine2D(filteredPointOfRotation, filteredAxisOfRotation);

      lineOfRotationStandardDeviation = new Line2DStatisticsCalculator(namePrefix + "LineOfRotation", lineOfRotationInSole, registry);

      if (graphicsListRegistry != null)
         edgeVisualizer = new EdgeVisualizer(namePrefix, Color.GREEN, registry, graphicsListRegistry);
      else
         edgeVisualizer = null;

      stabilityEvaluator = new EdgeVelocityStabilityEvaluator(namePrefix, lineOfRotationInSole, explorationParameters.getStableLoRAngularVelocityThreshold(),
                                                              explorationParameters.getStableCoRLinearVelocityThreshold(), dt, registry);

      reset();

      parentRegistry.addChild(registry);
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
   public void compute(FramePoint2DReadOnly measuredCoP)
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
         edgeVisualizer.visualize(stabilityEvaluator.isEdgeVelocityStable());
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
      return stabilityEvaluator.isEdgeVelocityStable();
   }
}
