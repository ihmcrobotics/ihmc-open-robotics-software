package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.DeadzoneYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

public class ClippedSpeedOffsetErrorInterpolator
{
   private static final double MAX_TRANSLATIONAL_CORRECTION_SPEED = 0.05;
   private static final double MAX_ROTATIONAL_CORRECTION_SPEED = 0.05;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final BooleanYoVariable isRotationCorrectionEnabled;

   private final YoVariableRegistry registry;

   private final BooleanYoVariable hasBeenCalled;

   private final DoubleYoVariable alphaFilter_BreakFrequency;
   private final DoubleYoVariable dt;

   ////////////////////////////////////////////
   private final FramePose referenceFrameToBeCorrectedPose_Translation = new FramePose(worldFrame);
   private final PoseReferenceFrame referenceFrameToBeCorrectedPoseReferenceFrame_Translation = new PoseReferenceFrame("referenceFrameToBeCorrectedPoseReferenceFrame_Translation", referenceFrameToBeCorrectedPose_Translation);
   private final FramePose referenceFrameToBeCorrectedPose_Rotation = new FramePose(worldFrame);
   private final PoseReferenceFrame referenceFrameToBeCorrectedPoseReferenceFrame_Rotation = new PoseReferenceFrame("referenceFrameToBeCorrectedPoseReferenceFrame_Rotation", referenceFrameToBeCorrectedPose_Rotation);

   private final RigidBodyTransform updatedStartOffsetTransform = new RigidBodyTransform();
   private final FramePose startOffsetErrorPose = new FramePose(worldFrame);
   private final Vector3d updatedStartOffset_Translation = new Vector3d();
   private final FrameOrientation updatedStartOffset_Rotation = new FrameOrientation(worldFrame);
   private final Quat4d updatedStartOffset_Rotation_quat = new Quat4d(0.0, 0.0, 0.0, 1.0);
   private final RigidBodyTransform startOffsetTransform_Translation = new RigidBodyTransform();
   private final RigidBodyTransform startOffsetTransform_Rotation = new RigidBodyTransform();

   private final Vector3d interpolatedTranslation = new Vector3d();
   private final RigidBodyTransform interpolatedTransform_Translation = new RigidBodyTransform();
   private final RigidBodyTransform interpolatedTransform_Rotation = new RigidBodyTransform();
   private final RigidBodyTransform interpolatedTransform = new RigidBodyTransform();
   private final Quat4d interpolatedRotation = new Quat4d(0.0, 0.0, 0.0, 1.0);

   private final RigidBodyTransform updatedGoalOffsetTransform = new RigidBodyTransform();
   private final FramePose goalOffsetErrorPose = new FramePose(worldFrame);
   private final Vector3d updatedGoalOffset_Translation = new Vector3d();
   private final FrameOrientation updatedGoalOffset_Rotation = new FrameOrientation(worldFrame);
   private final Quat4d updatedGoalOffset_Rotation_quat = new Quat4d(0.0, 0.0, 0.0, 1.0);
   private final RigidBodyTransform goalOffsetTransform_Translation = new RigidBodyTransform();
   private final RigidBodyTransform goalOffsetTransform_Rotation = new RigidBodyTransform();
   ////////////////////////////////////////////

   private final AlphaFilteredYoVariable alphaFilter;
   private final DoubleYoVariable alphaFilter_AlphaValue;
   private final DoubleYoVariable alphaFilter_PositionValue;
   private final DoubleYoVariable cLippedAlphaFilterValue;
   private final DoubleYoVariable previousClippedAlphaFilterValue;

   private final DoubleYoVariable maxTranslationalCorrectionVelocity;
   private final DoubleYoVariable maxRotationalCorrectionVelocity;

   private final Vector3d distanceToTravelVector = new Vector3d();
   private final DoubleYoVariable distanceToTravel;

   private final FrameOrientation rotationToTravel = new FrameOrientation(worldFrame);
   private final DoubleYoVariable angleToTravel;
   private final AxisAngle4d axisAngletoTravel = new AxisAngle4d();

   private final DoubleYoVariable translationalSpeedForGivenDistanceToTravel;
   private final DoubleYoVariable rotationalSpeedForGivenAngleToTravel;

   private final DoubleYoVariable temporaryTranslationAlphaClipped;
   private final DoubleYoVariable temporaryRotationAlphaClipped;

   private final ReferenceFrame referenceFrameToBeCorrected;
   private final RigidBodyTransform referenceFrameToBeCorrectedTransform_Translation = new RigidBodyTransform();
   private final RigidBodyTransform referenceFrameToBeCorrectedTransform_Rotation = new RigidBodyTransform();

   private final DoubleYoVariable xDeadzoneSize;
   private final DoubleYoVariable yDeadzoneSize;
   private final DoubleYoVariable zDeadzoneSize;

   private final DeadzoneYoVariable goalTranslationWithDeadzoneX;
   private final DeadzoneYoVariable goalTranslationWithDeadzoneY;
   private final DeadzoneYoVariable goalTranslationWithDeadzoneZ;

   private final DoubleYoVariable goalTranslationRawX;
   private final DoubleYoVariable goalTranslationRawY;
   private final DoubleYoVariable goalTranslationRawZ;

   private final Vector3d offsetBetweenStartAndGoalVector_Translation = new Vector3d();
   private final Vector3d updatedGoalOffsetWithDeadzone_Translation = new Vector3d();

   //used to check if the orientation error is too big
   private final PoseReferenceFrame correctedPelvisPoseReferenceFrame = new PoseReferenceFrame("correctedPelvisPoseReferenceFrame", worldFrame);
   private final FrameOrientation iterativeClosestPointOrientation = new FrameOrientation();
   private final double[] yawPitchRoll = new double[3];
   private final DoubleYoVariable maximumErrorAngleInDegrees;

   //for feedBack in scs
   private final YoFramePose yoStartOffsetErrorPose_InWorldFrame;
   private final YoFramePose yoGoalOffsetErrorPose_InWorldFrame;
   private final YoFramePose yoInterpolatedOffset_InWorldFrame;

   private final FramePose startOffsetErrorPose_Translation = new FramePose(worldFrame);
   private final FramePose startOffsetErrorPose_Rotation = new FramePose(worldFrame);
   private final PoseReferenceFrame startOffsetErrorReferenceFrame_Translation = new PoseReferenceFrame("startOffsetErrorReferenceFrame_Translation", startOffsetErrorPose_Translation);
   private final PoseReferenceFrame startOffsetErrorReferenceFrame_Rotation= new PoseReferenceFrame("startOffsetErrorReferenceFrame_Rotation", startOffsetErrorPose_Rotation);

   private final FramePoint goalOffsetFramePoint_Translation = new FramePoint(worldFrame);
   private final FramePoint interpolatedOffsetFramePoint_Translation = new FramePoint(worldFrame);
   private final FrameOrientation goalOffsetFrameOrientation_Rotation = new FrameOrientation(worldFrame);
   private final FrameOrientation interpolatedOffsetFrameOrientation_Rotation = new FrameOrientation(worldFrame);

   private final YoFramePoint yoGoalOffsetFramePoint_Translation;
   private final YoFramePoint yoInterpolatedOffsetFramePoint_Translation;

   private final YoFrameOrientation yoGoalOffsetFrameOrientation_Rotation;
   private final YoFrameOrientation yoInterpolatedOffsetFrameOrientation_Rotation;

   public ClippedSpeedOffsetErrorInterpolator(YoVariableRegistry parentRegistry, ReferenceFrame referenceFrame, DoubleYoVariable alphaFilterBreakFrequency,
         double estimator_dt, boolean correctRotation)
   {
      this.registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.alphaFilter_BreakFrequency = alphaFilterBreakFrequency;
      this.dt = new DoubleYoVariable("dt", registry);
      this.dt.set(estimator_dt);

      this.referenceFrameToBeCorrected = referenceFrame;

      isRotationCorrectionEnabled = new BooleanYoVariable("isRotationCorrectionEnabled", registry);
      isRotationCorrectionEnabled.set(correctRotation);

      hasBeenCalled = new BooleanYoVariable("hasbeenCalled", registry);
      hasBeenCalled.set(false);

      alphaFilter_AlphaValue = new DoubleYoVariable("alphaFilter_AlphaValue", registry);
      alphaFilter_AlphaValue.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(this.alphaFilter_BreakFrequency.getDoubleValue(),
            this.dt.getDoubleValue()));
      alphaFilter_PositionValue = new DoubleYoVariable("alphaFilter_PositionValue", registry);
      alphaFilter_PositionValue.set(0.0);
      alphaFilter = new AlphaFilteredYoVariable("alphaFilter", registry, alphaFilter_AlphaValue, alphaFilter_PositionValue);

      cLippedAlphaFilterValue = new DoubleYoVariable("cLippedAlphaFilterValue", registry);
      cLippedAlphaFilterValue.set(0.0);
      previousClippedAlphaFilterValue = new DoubleYoVariable("previousClippedAlphaFilterValue", registry);
      previousClippedAlphaFilterValue.set(0.0);

      maxTranslationalCorrectionVelocity = new DoubleYoVariable("maxTranslationalCorrectionVelocity", parentRegistry);
      maxTranslationalCorrectionVelocity.set(MAX_TRANSLATIONAL_CORRECTION_SPEED);
      maxRotationalCorrectionVelocity = new DoubleYoVariable("maxRotationalCorrectionVelocity", parentRegistry);
      maxRotationalCorrectionVelocity.set(MAX_ROTATIONAL_CORRECTION_SPEED);

      distanceToTravel = new DoubleYoVariable("distanceToTravel", registry);
      distanceToTravel.set(0.0);
      angleToTravel = new DoubleYoVariable("angleToTravel", registry);
      angleToTravel.set(0.0);

      translationalSpeedForGivenDistanceToTravel = new DoubleYoVariable("translationalSpeedForGivenDistanceToTravel", registry);
      rotationalSpeedForGivenAngleToTravel = new DoubleYoVariable("rotationalSpeedForGivenAngleToTravel", registry);

      temporaryTranslationAlphaClipped = new DoubleYoVariable("temporaryTranslationAlphaClipped", registry);
      temporaryRotationAlphaClipped = new DoubleYoVariable("temporaryRotationAlphaClipped", registry);

      xDeadzoneSize = new DoubleYoVariable("xDeadzoneSize", registry);
      xDeadzoneSize.set(0.014);
      yDeadzoneSize = new DoubleYoVariable("yDeadzoneSize", registry);
      yDeadzoneSize.set(0.014);
      zDeadzoneSize = new DoubleYoVariable("zDeadzoneSize", registry);
      zDeadzoneSize.set(0.014);

      goalTranslationRawX = new DoubleYoVariable("goalTranslationRawX", registry);
      goalTranslationRawY = new DoubleYoVariable("goalTranslationRawY", registry);
      goalTranslationRawZ = new DoubleYoVariable("goalTranslationRawZ", registry);

      goalTranslationWithDeadzoneX = new DeadzoneYoVariable("goalTranslationWithDeadzoneX", goalTranslationRawX, xDeadzoneSize, registry);
      goalTranslationWithDeadzoneY = new DeadzoneYoVariable("goalTranslationWithDeadzoneY", goalTranslationRawY, yDeadzoneSize, registry);
      goalTranslationWithDeadzoneZ = new DeadzoneYoVariable("goalTranslationWithDeadzoneZ", goalTranslationRawZ, zDeadzoneSize, registry);

      // for feedback in SCS
      yoStartOffsetErrorPose_InWorldFrame = new YoFramePose("yoStartOffsetErrorPose_InWorldFrame", worldFrame, registry);
      yoGoalOffsetErrorPose_InWorldFrame = new YoFramePose("yoGoalOffsetErrorPose_InWorldFrame", worldFrame, registry);
      yoInterpolatedOffset_InWorldFrame = new YoFramePose("yoInterpolatedOffset_InWorldFrame", worldFrame, registry);

      yoGoalOffsetFramePoint_Translation = new YoFramePoint("yoGoalOffsetFramePoint_Translation", startOffsetErrorReferenceFrame_Translation, registry);
      yoInterpolatedOffsetFramePoint_Translation = new YoFramePoint("yoInterpolatedOffsetFramePoint_Translation", startOffsetErrorReferenceFrame_Translation, registry);

      yoGoalOffsetFrameOrientation_Rotation = new YoFrameOrientation("yoGoalOffsetFrameOrientation_Rotation", startOffsetErrorReferenceFrame_Rotation, registry);
     yoInterpolatedOffsetFrameOrientation_Rotation = new YoFrameOrientation("yoInterpolatedOffsetFrameOrientation_Rotation", startOffsetErrorReferenceFrame_Rotation, registry);

      this.alphaFilter_BreakFrequency.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            alphaFilter_AlphaValue.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(alphaFilter_BreakFrequency.getDoubleValue(),
                  dt.getDoubleValue()));
         }
      });

      maximumErrorAngleInDegrees = new DoubleYoVariable("maximumErrorAngleInDegrees", registry);
      maximumErrorAngleInDegrees.set(10.0);
   }

   public boolean checkIfRotationErrorIsTooBig(FramePose correctedPelvisPoseInWorldFrame, FramePose iterativeClosestPointInWorldFramePose)
   {
      correctedPelvisPoseReferenceFrame.setPoseAndUpdate(correctedPelvisPoseInWorldFrame);
      iterativeClosestPointInWorldFramePose.getOrientationIncludingFrame(iterativeClosestPointOrientation);
      iterativeClosestPointOrientation.changeFrame(correctedPelvisPoseReferenceFrame);

      iterativeClosestPointOrientation.getYawPitchRoll(yawPitchRoll);

      for (int i = 0; i < yawPitchRoll.length; i++)
      {
         if (Math.abs(yawPitchRoll[i]) > Math.toRadians(maximumErrorAngleInDegrees.getDoubleValue()))
            return true;
      }
      return false;
   }

   public void setInterpolatorInputs(FramePose startOffsetError, FramePose goalOffsetError, double alphaFilterPosition)
   {
      startOffsetErrorPose.setPoseIncludingFrame(startOffsetError);
      goalOffsetErrorPose.setPoseIncludingFrame(goalOffsetError);
      if (!isRotationCorrectionEnabled.getBooleanValue())
      {
         startOffsetErrorPose.setOrientation(0.0, 0.0, 0.0);
         goalOffsetErrorPose.setOrientation(0.0, 0.0, 0.0);
      }

      //scs feedback only
      yoStartOffsetErrorPose_InWorldFrame.set(startOffsetErrorPose);
      yoGoalOffsetErrorPose_InWorldFrame.set(goalOffsetErrorPose);
      ///////////

      referenceFrameToBeCorrected.update();
      referenceFrameToBeCorrected.getTransformToDesiredFrame(referenceFrameToBeCorrectedTransform_Translation, worldFrame);
      referenceFrameToBeCorrectedTransform_Translation.setRotationToIdentity();
      referenceFrameToBeCorrected.getTransformToDesiredFrame(referenceFrameToBeCorrectedTransform_Rotation, worldFrame);
      referenceFrameToBeCorrectedTransform_Rotation.zeroTranslation();

      referenceFrameToBeCorrectedPose_Translation.setPose(referenceFrameToBeCorrectedTransform_Translation);
      referenceFrameToBeCorrectedPoseReferenceFrame_Translation.setPoseAndUpdate(referenceFrameToBeCorrectedPose_Translation);

      referenceFrameToBeCorrectedPose_Rotation.setPose(referenceFrameToBeCorrectedTransform_Rotation);
      referenceFrameToBeCorrectedPoseReferenceFrame_Rotation.setPoseAndUpdate(referenceFrameToBeCorrectedPose_Rotation);

      startOffsetErrorPose.changeFrame(referenceFrameToBeCorrectedPoseReferenceFrame_Translation);
      startOffsetErrorPose.getPosition(updatedStartOffset_Translation);
      startOffsetErrorPose.changeFrame(referenceFrameToBeCorrectedPoseReferenceFrame_Rotation);
      startOffsetErrorPose.getOrientation(updatedStartOffset_Rotation_quat);
      startOffsetErrorPose.changeFrame(worldFrame);
      startOffsetErrorPose.getOrientationIncludingFrame(updatedStartOffset_Rotation);

      goalOffsetErrorPose.changeFrame(referenceFrameToBeCorrectedPoseReferenceFrame_Translation);
      goalOffsetErrorPose.getPosition(updatedGoalOffset_Translation);
      goalOffsetErrorPose.changeFrame(referenceFrameToBeCorrectedPoseReferenceFrame_Rotation);
      goalOffsetErrorPose.getOrientation(updatedGoalOffset_Rotation_quat);
      goalOffsetErrorPose.changeFrame(worldFrame);
      goalOffsetErrorPose.getOrientationIncludingFrame(updatedGoalOffset_Rotation);

      startOffsetTransform_Translation.setTranslationAndIdentityRotation(updatedStartOffset_Translation);
      startOffsetTransform_Rotation.setRotationAndZeroTranslation(updatedStartOffset_Rotation_quat);

      offsetBetweenStartAndGoalVector_Translation.sub(updatedGoalOffset_Translation, updatedStartOffset_Translation);
      goalTranslationRawX.set(offsetBetweenStartAndGoalVector_Translation.getX());
      goalTranslationRawY.set(offsetBetweenStartAndGoalVector_Translation.getY());
      goalTranslationRawZ.set(offsetBetweenStartAndGoalVector_Translation.getZ());
      goalTranslationWithDeadzoneX.update();
      goalTranslationWithDeadzoneY.update();
      goalTranslationWithDeadzoneZ.update();

      updatedGoalOffsetWithDeadzone_Translation.setX(updatedStartOffset_Translation.getX() + goalTranslationWithDeadzoneX.getDoubleValue());
      updatedGoalOffsetWithDeadzone_Translation.setY(updatedStartOffset_Translation.getY() + goalTranslationWithDeadzoneY.getDoubleValue());
      updatedGoalOffsetWithDeadzone_Translation.setZ(updatedStartOffset_Translation.getZ() + goalTranslationWithDeadzoneZ.getDoubleValue());

      goalOffsetTransform_Translation.setTranslationAndIdentityRotation(updatedGoalOffsetWithDeadzone_Translation);
      goalOffsetTransform_Rotation.setRotationAndZeroTranslation(updatedGoalOffset_Rotation_quat);

      alphaFilter_PositionValue.set(alphaFilterPosition);
      alphaFilter.set(0.0);
      previousClippedAlphaFilterValue.set(0.0);
      cLippedAlphaFilterValue.set(0.0);

      updateMaxAlphaVariationSpeed();

      hasBeenCalled.set(false);
   }

   private void updateMaxAlphaVariationSpeed()
   {
      distanceToTravelVector.sub(updatedGoalOffsetWithDeadzone_Translation, updatedStartOffset_Translation);

      distanceToTravel.set(distanceToTravelVector.length());
      translationalSpeedForGivenDistanceToTravel.set(distanceToTravel.getDoubleValue() / dt.getDoubleValue());

      rotationToTravel.setOrientationFromOneToTwo(updatedStartOffset_Rotation, updatedGoalOffset_Rotation);
      rotationToTravel.getAxisAngle(axisAngletoTravel);

      angleToTravel.set(axisAngletoTravel.getAngle());
      rotationalSpeedForGivenAngleToTravel.set(angleToTravel.getDoubleValue() / dt.getDoubleValue());
   }

   public void interpolateError(FramePose offsetPoseToPack)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         alphaFilter.update(0.0);
         hasBeenCalled.set(true);
      }
      else
      {
         alphaFilter.update();
      }

      //translation
      if (translationalSpeedForGivenDistanceToTravel.getDoubleValue() * alphaFilter.getDoubleValue() <= translationalSpeedForGivenDistanceToTravel
            .getDoubleValue()
            * previousClippedAlphaFilterValue.getDoubleValue()
            + (maxTranslationalCorrectionVelocity.getDoubleValue() * dt.getDoubleValue() / distanceToTravel.getDoubleValue()))
      {
         temporaryTranslationAlphaClipped.set(alphaFilter.getDoubleValue());
      }
      else
      {
         temporaryTranslationAlphaClipped.set(previousClippedAlphaFilterValue.getDoubleValue()
               + (maxTranslationalCorrectionVelocity.getDoubleValue() * dt.getDoubleValue() / distanceToTravel.getDoubleValue()));
      }

      //rotation
      if (rotationalSpeedForGivenAngleToTravel.getDoubleValue() * alphaFilter.getDoubleValue() <= rotationalSpeedForGivenAngleToTravel.getDoubleValue()
            * previousClippedAlphaFilterValue.getDoubleValue() + (maxRotationalCorrectionVelocity.getDoubleValue() * dt.getDoubleValue() / angleToTravel.getDoubleValue()))
      {
         temporaryRotationAlphaClipped.set(alphaFilter.getDoubleValue());
      }
      else
      {
         temporaryRotationAlphaClipped.set(previousClippedAlphaFilterValue.getDoubleValue()
               + (maxRotationalCorrectionVelocity.getDoubleValue() * dt.getDoubleValue() / angleToTravel.getDoubleValue()));
      }

      //chose the smaller alpha (so the slower correction) between translation and rotation
      cLippedAlphaFilterValue.set(Math.min(temporaryTranslationAlphaClipped.getDoubleValue(), temporaryRotationAlphaClipped.getDoubleValue()));
      previousClippedAlphaFilterValue.set(cLippedAlphaFilterValue.getDoubleValue());

      referenceFrameToBeCorrected.getTransformToDesiredFrame(referenceFrameToBeCorrectedTransform_Translation, worldFrame);
      referenceFrameToBeCorrectedTransform_Translation.setRotationToIdentity();
      referenceFrameToBeCorrected.getTransformToDesiredFrame(referenceFrameToBeCorrectedTransform_Rotation, worldFrame);
      referenceFrameToBeCorrectedTransform_Rotation.zeroTranslation();

      //update the start and goal translations and rotations
      updatedStartOffsetTransform.setIdentity();
      updatedStartOffsetTransform.multiply(referenceFrameToBeCorrectedTransform_Translation);
      updatedStartOffsetTransform.multiply(startOffsetTransform_Translation);
      updatedStartOffsetTransform.multiply(referenceFrameToBeCorrectedTransform_Rotation);
      updatedStartOffsetTransform.multiply(startOffsetTransform_Rotation);

      updatedStartOffsetTransform.getTranslation(updatedStartOffset_Translation);
      updatedStartOffsetTransform.getRotation(updatedStartOffset_Rotation_quat);

      updatedGoalOffsetTransform.setIdentity();
      updatedGoalOffsetTransform.multiply(referenceFrameToBeCorrectedTransform_Translation);
      updatedGoalOffsetTransform.multiply(goalOffsetTransform_Translation);
      updatedGoalOffsetTransform.multiply(referenceFrameToBeCorrectedTransform_Rotation);
      updatedGoalOffsetTransform.multiply(goalOffsetTransform_Rotation);

      updatedGoalOffsetTransform.getTranslation(updatedGoalOffset_Translation);
      updatedGoalOffsetTransform.getRotation(updatedGoalOffset_Rotation_quat);

      //interpolation here
      interpolatedTranslation.interpolate(updatedStartOffset_Translation, updatedGoalOffset_Translation, cLippedAlphaFilterValue.getDoubleValue());
      interpolatedTransform_Translation.setTranslationAndIdentityRotation(interpolatedTranslation);

      if (isRotationCorrectionEnabled.getBooleanValue())
      {
         interpolatedRotation.interpolate(updatedStartOffset_Rotation_quat, updatedGoalOffset_Rotation_quat, cLippedAlphaFilterValue.getDoubleValue());
         interpolatedTransform_Rotation.setRotationAndZeroTranslation(interpolatedRotation);
      }
      else
      {
         interpolatedTransform_Rotation.set(referenceFrameToBeCorrectedTransform_Rotation);
      }

      interpolatedTransform.setIdentity();
      interpolatedTransform.multiply(interpolatedTransform_Translation);
      interpolatedTransform.multiply(interpolatedTransform_Rotation);

      offsetPoseToPack.setPose(interpolatedTransform);

      //scs feedback only
      yoStartOffsetErrorPose_InWorldFrame.setPosition(updatedStartOffset_Translation);
      yoStartOffsetErrorPose_InWorldFrame.setOrientation(updatedStartOffset_Rotation_quat);
      yoGoalOffsetErrorPose_InWorldFrame.setPosition(updatedGoalOffset_Translation);
      yoGoalOffsetErrorPose_InWorldFrame.setOrientation(updatedGoalOffset_Rotation_quat);
      yoInterpolatedOffset_InWorldFrame.set(offsetPoseToPack);

      //here we express the goal and interpolated translation and rotation with respect to the start Translation and rotation so that the graphs are easier to read in SCS 
      startOffsetErrorPose_Translation.setToZero(worldFrame);
      startOffsetErrorPose_Translation.setPosition(updatedStartOffset_Translation);
      startOffsetErrorReferenceFrame_Translation.setPoseAndUpdate(startOffsetErrorPose_Translation);

      startOffsetErrorPose_Rotation.setToZero(worldFrame);
      startOffsetErrorPose_Rotation.setOrientation(updatedStartOffset_Rotation);
      startOffsetErrorReferenceFrame_Rotation.setPoseAndUpdate(startOffsetErrorPose_Rotation);

      goalOffsetFramePoint_Translation.changeFrame(worldFrame);
      goalOffsetFramePoint_Translation.set(updatedGoalOffset_Translation);
      goalOffsetFramePoint_Translation.changeFrame(startOffsetErrorReferenceFrame_Translation);
      yoGoalOffsetFramePoint_Translation.setAndMatchFrame(goalOffsetFramePoint_Translation);

      interpolatedOffsetFramePoint_Translation.changeFrame(worldFrame);
      interpolatedOffsetFramePoint_Translation.set(interpolatedTranslation);
      interpolatedOffsetFramePoint_Translation.changeFrame(startOffsetErrorReferenceFrame_Translation);
      yoInterpolatedOffsetFramePoint_Translation.setAndMatchFrame(interpolatedOffsetFramePoint_Translation);

      goalOffsetFrameOrientation_Rotation.changeFrame(worldFrame);
      goalOffsetFrameOrientation_Rotation.set(updatedGoalOffset_Rotation);
      goalOffsetFrameOrientation_Rotation.changeFrame(startOffsetErrorReferenceFrame_Rotation);
      yoGoalOffsetFrameOrientation_Rotation.setAndMatchFrame(goalOffsetFrameOrientation_Rotation);

      interpolatedOffsetFrameOrientation_Rotation.changeFrame(worldFrame);
      interpolatedOffsetFrameOrientation_Rotation.set(interpolatedRotation);
      interpolatedOffsetFrameOrientation_Rotation.changeFrame(startOffsetErrorReferenceFrame_Rotation);
      yoInterpolatedOffsetFrameOrientation_Rotation.setAndMatchFrame(interpolatedOffsetFrameOrientation_Rotation);
   }

   public void setDeadZoneSizes(double xDeadzoneSize, double yDeadzoneSize, double zDeadzoneSize)
   {
      this.xDeadzoneSize.set(xDeadzoneSize);
      this.yDeadzoneSize.set(yDeadzoneSize);
      this.zDeadzoneSize.set(zDeadzoneSize);
   }
}
