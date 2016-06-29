package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.awt.Color;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPosition;

public class AdjustedCapturePointCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FramePoint2d icpAdjusted = new FramePoint2d(worldFrame);

   private final double gravity;
   private final double mass;
   private final double omega0;

   private final FramePoint2d localCop = new FramePoint2d();
   private final FramePoint2d localIcp = new FramePoint2d();

   private final YoFramePoint2d yoIcpAdjusted = new YoFramePoint2d("IcpAdjusted", worldFrame, registry);
   private final YoFramePoint2d yoIcpOriginal = new YoFramePoint2d("IcpOriginal", worldFrame, registry);
   private final YoFrameVector2d yoAngularMomentum = new YoFrameVector2d("AngularMomentum", worldFrame, registry);

   private final DoubleYoVariable predictionAlpha = new DoubleYoVariable("PreditionAlpha", registry);

   public AdjustedCapturePointCalculator(double gravity, double mass, double omega0,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.gravity = gravity;
      this.mass = mass;
      this.omega0 = omega0;

      predictionAlpha.set(0.75);

      if (yoGraphicsListRegistry != null)
      {
         String graphicsName = getClass().getSimpleName();

         YoArtifactPosition icpArtifact = new YoArtifactPosition("Adjusted Capture Point", yoIcpAdjusted.getYoX(), yoIcpAdjusted.getYoY(), GraphicType.ROTATED_CROSS, Color.blue, 0.005);
         yoGraphicsListRegistry.registerArtifact(graphicsName, icpArtifact);

         YoArtifactPosition copArtifact = new YoArtifactPosition("Original Capture Point", yoIcpOriginal.getYoX(), yoIcpOriginal.getYoY(), GraphicType.ROTATED_CROSS, Color.blue, 0.01);
         yoGraphicsListRegistry.registerArtifact(graphicsName, copArtifact);
      }

      parentRegistry.addChild(registry);
   }

   public void computeAdjustedCapturePoint(double timeToStop, FramePoint2d cop, FramePoint2d icp, FrameVector2d angularMomentum, FramePoint2d icpAdjustedToPack)
   {
      ReferenceFrame comFrame = angularMomentum.getReferenceFrame();

      localCop.setIncludingFrame(cop);
      localCop.changeFrameAndProjectToXYPlane(comFrame);
      localIcp.setIncludingFrame(icp);
      localIcp.changeFrameAndProjectToXYPlane(comFrame);

//      cmpAdjusted.setIncludingFrame(localAngularMomentum);
//      cmpAdjusted.scale(-1.0 / (timeToStop * gravity * mass));
//      cmpAdjusted.add(localCop);
//      CaptureRegionMathTools.predictCapturePoint(localIcp, cmpAdjusted, timeToStop, omega0, icpAdjusted);

      // icpAdjusted = icp - L * omega/(m*g)
      icpAdjusted.setIncludingFrame(comFrame, angularMomentum.getY(), angularMomentum.getX());
      icpAdjusted.scale(- omega0 / (mass * gravity));
      icpAdjusted.add(localIcp);

      icpAdjusted.changeFrameAndProjectToXYPlane(worldFrame);
      icpAdjustedToPack.interpolate(icpAdjusted, icp, predictionAlpha.getDoubleValue());
//      icpAdjustedToPack.setIncludingFrame(icpAdjusted);

      yoIcpAdjusted.set(icpAdjustedToPack);
      yoIcpOriginal.set(icp);
      yoAngularMomentum.set(angularMomentum.getX(), angularMomentum.getY());
   }
}
