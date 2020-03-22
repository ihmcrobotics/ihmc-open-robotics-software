package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

import java.awt.*;
import java.util.EnumMap;
import java.util.List;

public class PartialFootholdCropperModule
{
   private final RotationEdgeCalculator copHistoryEdgeCalculator;
   private final CoPAndVelocityRotationEdgeCalculator copAndVelocityEdgeCalculator;
   private final FootholdCropper footholdCropper;
   private final CropVerifier cropVerifier;

   private final YoBoolean isEdgeStable;
   private final YoBoolean shouldShrinkFoothold;

   private final FootRotationDetector rotationDetector;

   private final EdgeVisualizer edgeVisualizer;

   public PartialFootholdCropperModule(RobotSide side,
                                       MovingReferenceFrame soleFrame,
                                       List<? extends FramePoint2DReadOnly> defaultContactPoints,
                                       FootholdRotationParameters rotationParameters,
                                       double dt,
                                       YoVariableRegistry parentRegistry,
                                       YoGraphicsListRegistry graphicsRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());
      parentRegistry.addChild(registry);

      isEdgeStable = new YoBoolean(side.getLowerCaseName() + "IsEdgeStable", registry);
      shouldShrinkFoothold = new YoBoolean(side.getLowerCaseName() + "ShouldShrinkFoothold", registry);

      rotationDetector = new CombinedFootRotationDetector(side, soleFrame, rotationParameters, dt, registry);

      copHistoryEdgeCalculator = new CoPHistoryRotationEdgeCalculator(side, soleFrame, rotationParameters, dt, registry, Color.BLUE, graphicsRegistry);
      copAndVelocityEdgeCalculator = new CoPAndVelocityRotationEdgeCalculator(side, soleFrame, rotationParameters, dt, registry, Color.GRAY, graphicsRegistry);


//      if (graphicsRegistry != null)
//         edgeVisualizer = new EdgeVisualizer(side.getLowerCaseName(), Color.RED, registry, graphicsRegistry);
//      else
         edgeVisualizer = null;

      cropVerifier = new CropVerifier(side.getLowerCaseName(), soleFrame, 0.005, rotationParameters, registry, graphicsRegistry);
      footholdCropper = new FootholdCropper(side.getLowerCaseName(), soleFrame, defaultContactPoints, rotationParameters, dt, registry, graphicsRegistry);

      reset();
   }

   public void compute(FramePoint2DReadOnly measuredCoP, FramePoint2DReadOnly desiredCoP)
   {
      boolean wasRotating = rotationDetector.isRotating();
      boolean isRotating = rotationDetector.compute();
      shouldShrinkFoothold.set(false);

      cropVerifier.update(desiredCoP);
      footholdCropper.update(measuredCoP);

      if (!isRotating)
      {
         if (wasRotating)
            resetEdgeCalculators();
         isEdgeStable.set(false);
         return;
      }

      FrameLine2DReadOnly lineOfRotation = computeLineOfRotation(measuredCoP);
      isEdgeStable.set(lineOfRotation != null);
      if (!isEdgeStable.getBooleanValue())
         return;

      if (edgeVisualizer != null)
      {
         edgeVisualizer.visualize(true);
         edgeVisualizer.updateGraphics(lineOfRotation);
      }

      RobotSide sideToCrop = footholdCropper.computeSideToCrop(lineOfRotation);
      if (sideToCrop != null)
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

   private FrameLine2DReadOnly computeLineOfRotation(FramePoint2DReadOnly measuredCoP)
   {
      copHistoryEdgeCalculator.compute(measuredCoP);
      copAndVelocityEdgeCalculator.compute(measuredCoP);

      return getLineOfRotation();
   }

   public FrameLine2DReadOnly getLineOfRotation()
   {
      if (copHistoryEdgeCalculator.isRotationEdgeTrusted())
         return copHistoryEdgeCalculator.getLineOfRotation();
      if (copAndVelocityEdgeCalculator.isRotationEdgeTrusted())
         return copAndVelocityEdgeCalculator.getLineOfRotation();
      else
         return null;
   }

   private void resetEdgeCalculators()
   {
      if (edgeVisualizer != null)
      {
         edgeVisualizer.visualize(false);
         edgeVisualizer.reset();
      }

      copAndVelocityEdgeCalculator.reset();
      copHistoryEdgeCalculator.reset();
   }

   public void initialize(FrameConvexPolygon2DReadOnly footPolygon)
   {
      cropVerifier.initialize();
      footholdCropper.reset(footPolygon);
   }

   public void reset()
   {
      isEdgeStable.set(false);
      rotationDetector.reset();
      resetEdgeCalculators();
      cropVerifier.reset();
      footholdCropper.reset();
   }
}
