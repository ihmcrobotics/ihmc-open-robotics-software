package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class FootDropCropper
{
   private final YoDouble footDropOrLift;
   private final YoDouble footDropOrLiftDelta;
   private final FrameVector3D pointingBackwardVector = new FrameVector3D();
   private final DoubleProvider footDropThreshold;

   private final YoDouble dropAtStartOfMeasurement;

   private final YoEnum<RobotSide> sideOfFootToCrop;
   private final ReferenceFrame soleFrame;

   public FootDropCropper(String namePrefix, ReferenceFrame soleFrame, FootholdRotationParameters rotationParameters, YoRegistry registry)
   {
      this.soleFrame = soleFrame;
      footDropThreshold = rotationParameters.getFootDropThresholdForCrop();

      dropAtStartOfMeasurement = new YoDouble(namePrefix + "CropDropAtStartOfMeasurement", registry);
      footDropOrLift = new YoDouble(namePrefix + "CropFootDropOrLift", registry);
      footDropOrLiftDelta = new YoDouble(namePrefix + "CropFootDropOrLiftDelta", registry);
      sideOfFootToCrop = new YoEnum<>(namePrefix + "CropDropSideOfFootToCrop", registry, RobotSide.class, true);
   }

   public void reset()
   {
      sideOfFootToCrop.set(null);
      footDropOrLift.setToNaN();
      dropAtStartOfMeasurement.setToNaN();
   }

   public RobotSide computeSideOfFootholdToCrop(FrameLine2DReadOnly lineOfRotation)
   {
      pointingBackwardVector.setIncludingFrame(lineOfRotation.getReferenceFrame(), lineOfRotation.getDirection().getY(), -lineOfRotation.getDirection().getX(), 0.0);
      pointingBackwardVector.changeFrame(soleFrame);
      pointingBackwardVector.setZ(0.0);

      pointingBackwardVector.normalize();
      pointingBackwardVector.scale(0.15);
      pointingBackwardVector.changeFrame(ReferenceFrame.getWorldFrame());

      if (Double.isNaN(dropAtStartOfMeasurement.getDoubleValue()))
         dropAtStartOfMeasurement.set(pointingBackwardVector.getZ());

      footDropOrLift.set(pointingBackwardVector.getZ());
      footDropOrLiftDelta.set(footDropOrLift.getValue() - dropAtStartOfMeasurement.getDoubleValue());

      if (footDropOrLiftDelta.getDoubleValue() < -footDropThreshold.getValue())
         sideOfFootToCrop.set(RobotSide.RIGHT);
      else if (footDropOrLiftDelta.getDoubleValue() > footDropThreshold.getValue())
         sideOfFootToCrop.set(RobotSide.LEFT);
      else
         sideOfFootToCrop.set(null);

      return sideOfFootToCrop.getEnumValue();
   }
}
