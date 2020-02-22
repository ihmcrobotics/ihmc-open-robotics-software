package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
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

   private final AlphaFilteredYoFramePoint2d filteredPointOfRotation;
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

      YoFramePoint2D point = new YoFramePoint2D(side.getLowerCaseName() + "LineOfRotationPoint", soleFrame, registry);
      YoFrameVector2D direction = new YoFrameVector2D(side.getLowerCaseName() + "LineOfRotationDirection", soleFrame, registry);
      lineOfRotationInSole = new YoFrameLine2D(point, direction);

      String feetManagerName = FeetManager.class.getSimpleName();
      String paramRegistryName = getClass().getSimpleName() + "Parameters";
      filterBreakFrequency = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "filterBreakFrequency", registry, 1.0);

      DoubleProvider alpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filterBreakFrequency.getValue(), dt);
      filteredPointOfRotation = new AlphaFilteredYoFramePoint2d(side + "FilteredPointOfRotation", "", registry, alpha, soleFrame);
      filteredAxisOfRotation = new AlphaFilteredYoFrameVector2d(side + "FilteredAxisOfRotation", "", registry, alpha, soleFrame);

      lineOfRotationStandardDeviation = new Line2DStatisticsCalculator(side.getLowerCaseName() + "LineOfRotation", lineOfRotationInSole, registry);

      if (graphicsListRegistry != null)
         edgeVisualizer = new EdgeVisualizer(side.getLowerCaseName() + "Velocity", registry, graphicsListRegistry);
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
      lineOfRotationInSole.setToZero();
      lineOfRotationInSole.getPoint().set(tempPointOfRotation);
      lineOfRotationInSole.getDirection().set(soleFrameTwist.getAngularPart());
      lineOfRotationInSole.getDirection().scale(1.0 / omega);

      if (lineOfRotationInSole.getDirection().dot(filteredAxisOfRotation) < 0.0)
      {
         lineOfRotationInSole.getDirection().negate();
      }

      // Filter the line of rotation:
      filteredPointOfRotation.update(lineOfRotationInSole.getPoint());
      filteredAxisOfRotation.update(lineOfRotationInSole.getDirection());
      lineOfRotationInSole.set(filteredPointOfRotation, filteredAxisOfRotation);
      lineOfRotationInSole.getDirection().normalize();

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
