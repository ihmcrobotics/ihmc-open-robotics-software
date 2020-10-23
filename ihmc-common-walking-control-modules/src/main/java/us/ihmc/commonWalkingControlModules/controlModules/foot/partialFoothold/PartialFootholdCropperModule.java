package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.List;

public class PartialFootholdCropperModule
{
   private final FootholdCropper footholdCropper;
   private final CropVerifier cropVerifier;

   private final YoBoolean shouldShrinkFoothold;

   private final FootRotationDetector rotationDetector;
   private final RotationEdgeCalculator edgeCalculator;

   public PartialFootholdCropperModule(RobotSide side,
                                       MovingReferenceFrame soleFrame,
                                       List<? extends FramePoint2DReadOnly> defaultContactPoints,
                                       FootholdRotationParameters rotationParameters,
                                       double dt,
                                       YoRegistry parentRegistry,
                                       YoGraphicsListRegistry graphicsRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName() + side.getPascalCaseName());
      parentRegistry.addChild(registry);

      shouldShrinkFoothold = new YoBoolean(side.getLowerCaseName() + "ShouldShrinkFoothold", registry);

      rotationDetector = new CombinedFootRotationDetector(side, soleFrame, rotationParameters, dt, registry);
      edgeCalculator = new CombinedRotationEdgeCalculator(side, soleFrame, rotationParameters, dt, registry, graphicsRegistry);

      cropVerifier = new CropVerifier(side.getLowerCaseName(), soleFrame, 0.005, rotationParameters, registry, null);
      footholdCropper = new FootholdCropper(side.getLowerCaseName(), soleFrame, defaultContactPoints, rotationParameters, dt, registry, graphicsRegistry);

      reset();
   }

   public void compute(FramePoint2DReadOnly measuredCoP, FramePoint2DReadOnly desiredCoP)
   {
      boolean isRotating = rotationDetector.compute();
      shouldShrinkFoothold.set(false);

      cropVerifier.update(desiredCoP);
      footholdCropper.update(measuredCoP);

      if (!isRotating)
      {
         edgeCalculator.reset();
         return;
      }

      boolean isEdgeTrusted = edgeCalculator.compute(measuredCoP);

      FrameLine2DReadOnly lineOfRotation = edgeCalculator.getLineOfRotation();
      RobotSide sideToCrop = footholdCropper.computeSideToCrop(lineOfRotation);
      
      if (isEdgeTrusted && sideToCrop != null)
      {
         shouldShrinkFoothold.set(cropVerifier.verifyFootholdCrop(desiredCoP, sideToCrop, lineOfRotation));
         if (shouldShrinkFoothold.getBooleanValue())
            footholdCropper.computeShrunkenFoothold(lineOfRotation, sideToCrop);
      }
   }

   public boolean isRotating()
   {
      return rotationDetector.compute();
   }

   public boolean applyShrunkenFoothold(YoPlaneContactState contactStateToModify)
   {
      if (!shouldApplyShrunkenFoothold())
         return false;

      return footholdCropper.applyShrunkenFoothold(contactStateToModify);
   }

   boolean shouldApplyShrunkenFoothold()
   {
      if (!shouldShrinkFoothold.getBooleanValue())
         return false;

      return footholdCropper.shouldApplyShrunkenFoothold();
   }

   public FrameConvexPolygon2DReadOnly getShrunkenFootPolygon()
   {
      return footholdCropper.getShrunkenFootPolygon();
   }

   public FrameLine2DReadOnly getLineOfRotation()
   {
      return edgeCalculator.getLineOfRotation();
   }

   public void initialize(FrameConvexPolygon2DReadOnly footPolygon)
   {
      cropVerifier.initialize();
      footholdCropper.reset(footPolygon);
   }

   public void reset()
   {
      rotationDetector.reset();
      edgeCalculator.reset();
      cropVerifier.reset();
      footholdCropper.reset();
   }
}
