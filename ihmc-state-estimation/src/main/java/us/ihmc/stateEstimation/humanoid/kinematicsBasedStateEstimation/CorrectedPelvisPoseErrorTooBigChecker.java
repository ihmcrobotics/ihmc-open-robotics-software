package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CorrectedPelvisPoseErrorTooBigChecker
{
   private static final double MAXIMUM_TRANSLATION_ERROR = 0.15;
   private static final double MAXIMUM_ANGLE_ERROR_IN_DEGRESS = 10.0;

   private final YoVariableRegistry registry;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final PoseReferenceFrame correctedPelvisPoseReferenceFrame = new PoseReferenceFrame("correctedPelvisPoseReferenceFrame", worldFrame);
   private final FrameQuaternion iterativeClosestPointOrientation = new FrameQuaternion();
   private final FramePoint3D iterativeClosestPointTranslation = new FramePoint3D();
   private final AxisAngle axisAngleForError = new AxisAngle();
   private final YoDouble maximumErrorAngleInDegrees;
   private final YoDouble maximumErrorTranslation;

   private final YoDouble angleError;
   private final YoDouble translationErrorX;
   private final YoDouble translationErrorY;
   private final YoDouble translationErrorZ;

   public CorrectedPelvisPoseErrorTooBigChecker(YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());

      maximumErrorAngleInDegrees = new YoDouble("maximumErrorAngleInDegrees", registry);
      maximumErrorAngleInDegrees.set(MAXIMUM_ANGLE_ERROR_IN_DEGRESS);
      maximumErrorTranslation = new YoDouble("maximumErrorTranslation", registry);
      maximumErrorTranslation.set(MAXIMUM_TRANSLATION_ERROR);

      angleError = new YoDouble("angleError", registry);
      translationErrorX = new YoDouble("translationErrorX", registry);
      translationErrorY = new YoDouble("translationErrorY", registry);
      translationErrorZ = new YoDouble("translationErrorZ", registry);

      parentRegistry.addChild(registry);
   }

   public boolean checkIfErrorIsTooBig(FramePose3D correctedPelvisPoseInWorldFrame, FramePose3D iterativeClosestPointInWorldFramePose,
                                       boolean isRotationCorrectionEnabled)
   {
      correctedPelvisPoseReferenceFrame.setPoseAndUpdate(correctedPelvisPoseInWorldFrame);

      iterativeClosestPointOrientation.setIncludingFrame(iterativeClosestPointInWorldFramePose.getOrientation());
      iterativeClosestPointTranslation.setIncludingFrame(iterativeClosestPointInWorldFramePose.getPosition());

      iterativeClosestPointOrientation.changeFrame(correctedPelvisPoseReferenceFrame);
      iterativeClosestPointTranslation.changeFrame(correctedPelvisPoseReferenceFrame);

      axisAngleForError.set(iterativeClosestPointOrientation);

      angleError.set(Math.abs(axisAngleForError.getAngle()));
      translationErrorX.set(Math.abs(iterativeClosestPointTranslation.getX()));
      translationErrorY.set(Math.abs(iterativeClosestPointTranslation.getY()));
      translationErrorZ.set(Math.abs(iterativeClosestPointTranslation.getZ()));

      if (isRotationCorrectionEnabled && Math.abs(axisAngleForError.getAngle()) > Math.toRadians(maximumErrorAngleInDegrees.getDoubleValue()))
         return true;

      if (Math.abs(iterativeClosestPointTranslation.getX()) > maximumErrorTranslation.getDoubleValue())
         return true;
      if (Math.abs(iterativeClosestPointTranslation.getY()) > maximumErrorTranslation.getDoubleValue())
         return true;
      if (Math.abs(iterativeClosestPointTranslation.getZ()) > maximumErrorTranslation.getDoubleValue())
         return true;

      return false;
   }
}
