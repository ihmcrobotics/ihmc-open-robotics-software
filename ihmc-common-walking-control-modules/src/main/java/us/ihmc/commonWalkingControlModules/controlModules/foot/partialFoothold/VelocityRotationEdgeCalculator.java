package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

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

   public VelocityRotationEdgeCalculator(RobotSide side, MovingReferenceFrame soleFrame, double dt, YoVariableRegistry parentRegistry,
                                         YoGraphicsListRegistry graphicsListRegistry)
   {
      this.soleFrame = soleFrame;

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());

      pointOfRotation = new YoFramePoint2D(side.getLowerCaseName() + "PointOfRotation", soleFrame, registry);
      axisOfRotation = new YoFrameVector2D(side.getLowerCaseName() + "AxisOfRotation", soleFrame, registry);

      String feetManagerName = FeetManager.class.getSimpleName();
      String paramRegistryName = getClass().getSimpleName() + "Parameters";
      filterBreakFrequency = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "filterBreakFrequency", registry, 1.0);

      DoubleProvider alpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filterBreakFrequency.getValue(), dt);
      filteredPointOfRotation = new AlphaFilteredYoFramePoint2d(side + "FilteredPointOfRotation", "", registry, alpha, pointOfRotation);
      filteredAxisOfRotation = new AlphaFilteredYoFrameVector2d(side + "FilteredAxisOfRotation", "", registry, alpha, axisOfRotation);

      lineOfRotationInSole = new YoFrameLine2D(filteredPointOfRotation, filteredAxisOfRotation);


      lineOfRotationStandardDeviation = new Line2DStatisticsCalculator(side.getLowerCaseName() + "LineOfRotation", lineOfRotationInSole, registry);

      if (graphicsListRegistry != null)
         edgeVisualizer = new EdgeVisualizer(side.getLowerCaseName() + "Velocity", Color.GREEN, registry, graphicsListRegistry);
      else
         edgeVisualizer = null;

      reset();

      parentRegistry.addChild(registry);
   }

   private final FrameVector3D tempPointOfRotation = new FrameVector3D();


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

      if (edgeVisualizer != null)
         edgeVisualizer.updateGraphics(lineOfRotationInSole);
   }

   public void reset()
   {
      if (edgeVisualizer != null)
         edgeVisualizer.reset();

      filteredPointOfRotation.reset();
      filteredAxisOfRotation.reset();
      lineOfRotationInSole.setToZero();

      lineOfRotationStandardDeviation.reset();
   }

   public FrameLine2DReadOnly getLineOfRotation()
   {
      return lineOfRotationInSole;
   }
}
