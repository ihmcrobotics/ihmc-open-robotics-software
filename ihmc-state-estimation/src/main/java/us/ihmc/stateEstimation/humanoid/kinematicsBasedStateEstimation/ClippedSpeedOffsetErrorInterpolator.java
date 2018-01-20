package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.DeadzoneYoVariable;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class ClippedSpeedOffsetErrorInterpolator
{
   private static final double Z_DEADZONE_SIZE = 0.014;
   private static final double Y_DEADZONE_SIZE = 0.014;
   private static final double X_DEADZONE_SIZE = 0.014;
   
   private static final double YAW_DEADZONE_IN_DEGREES = 1.0;
   
   private static final double MAXIMUM_TRANSLATION_ERROR = 0.15;
   private static final double MAXIMUM_ANGLE_ERROR_IN_DEGRESS = 10.0;
   
   private static final double MAX_TRANSLATIONAL_CORRECTION_SPEED = 0.05;
   private static final double MAX_ROTATIONAL_CORRECTION_SPEED = 0.05;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoBoolean isRotationCorrectionEnabled;

   private final YoVariableRegistry registry;

   private final YoBoolean hasBeenCalled;

   private final YoDouble alphaFilter_BreakFrequency;
   private final YoDouble dt;

   ////////////////////////////////////////////
   private final FramePose3D stateEstimatorPose_Translation = new FramePose3D(worldFrame);
   private final PoseReferenceFrame stateEstimatorReferenceFrame_Translation = new PoseReferenceFrame("stateEstimatorReferenceFrame_Translation", stateEstimatorPose_Translation);
   private final FramePose3D stateEstimatorPose_Rotation = new FramePose3D(worldFrame);
   private final PoseReferenceFrame stateEstimatorReferenceFrame_Rotation = new PoseReferenceFrame("stateEstimatorReferenceFrame_Rotation", stateEstimatorPose_Rotation);

   private final RigidBodyTransform updatedStartOffsetTransform = new RigidBodyTransform();
   private final FramePose3D startOffsetErrorPose = new FramePose3D(worldFrame);
   private final Vector3D updatedStartOffset_Translation = new Vector3D();
   private final FrameQuaternion updatedStartOffset_Rotation = new FrameQuaternion(worldFrame);
   private final Quaternion updatedStartOffset_Rotation_quat = new Quaternion(0.0, 0.0, 0.0, 1.0);
   private final RigidBodyTransform startOffsetTransform_Translation = new RigidBodyTransform();
   private final RigidBodyTransform startOffsetTransform_Rotation = new RigidBodyTransform();

   private final Vector3D interpolatedTranslation = new Vector3D();
   private final Quaternion interpolatedRotation = new Quaternion(0.0, 0.0, 0.0, 1.0);

   private final RigidBodyTransform updatedGoalOffsetTransform = new RigidBodyTransform();
   private final FramePose3D goalOffsetErrorPose = new FramePose3D(worldFrame);
   private final Vector3D updatedGoalOffset_Translation = new Vector3D();
   private final FrameQuaternion updatedGoalOffset_Rotation = new FrameQuaternion(worldFrame);
   private final Quaternion updatedGoalOffset_Rotation_quat = new Quaternion(0.0, 0.0, 0.0, 1.0);
   private final RigidBodyTransform goalOffsetTransform_Translation = new RigidBodyTransform();
   private final RigidBodyTransform goalOffsetTransform_Rotation = new RigidBodyTransform();
   
   ////////////////////////////////////////////

   private final AlphaFilteredYoVariable alphaFilter;
   private final YoDouble alphaFilter_AlphaValue;
   private final YoDouble alphaFilter_PositionValue;
   private final YoDouble cLippedAlphaFilterValue;
   private final YoDouble previousClippedAlphaFilterValue;

   private final YoDouble maxTranslationalCorrectionVelocity;
   private final YoDouble maxRotationalCorrectionVelocity;

   private final Vector3D distanceToTravelVector = new Vector3D();
   private final YoDouble distanceToTravel;

   private final FrameQuaternion rotationToTravel = new FrameQuaternion(worldFrame);
   private final YoDouble angleToTravel;
   private final AxisAngle axisAngletoTravel = new AxisAngle();

   private final YoDouble translationalSpeedForGivenDistanceToTravel;
   private final YoDouble rotationalSpeedForGivenAngleToTravel;

   private final YoDouble temporaryTranslationAlphaClipped;
   private final YoDouble temporaryRotationAlphaClipped;

   private final ReferenceFrame stateEstimatorReferenceFrame;
   private final RigidBodyTransform stateEstimatorTransform_Translation = new RigidBodyTransform();
   private final RigidBodyTransform stateEstimatorTransform_Rotation = new RigidBodyTransform();

   
   //Deadzone translation variables
   private final YoDouble xDeadzoneSize;
   private final YoDouble yDeadzoneSize;
   private final YoDouble zDeadzoneSize;
   private final DeadzoneYoVariable goalTranslationWithDeadzoneX;
   private final DeadzoneYoVariable goalTranslationWithDeadzoneY;
   private final DeadzoneYoVariable goalTranslationWithDeadzoneZ;
   private final YoDouble goalTranslationRawX;
   private final YoDouble goalTranslationRawY;
   private final YoDouble goalTranslationRawZ;
   private final Vector3D offsetBetweenStartAndGoalVector_Translation = new Vector3D();
   private final Vector3D updatedGoalOffsetWithDeadzone_Translation = new Vector3D();
   
   //Deadzone rotation Variables
   private final YoDouble yawDeadzoneSize;
   private final DeadzoneYoVariable goalYawWithDeadZone;
   private final YoDouble goalYawRaw;
   private final FrameQuaternion offsetBetweenStartAndGoal_Rotation = new FrameQuaternion(worldFrame);
   private final FrameQuaternion updatedGoalOffsetWithDeadZone_Rotation = new FrameQuaternion(worldFrame);
   private final Quaternion updatedGoalOffsetWithDeadZone_Rotation_quat = new Quaternion();
   
   double[] stateEstimatorYawPitchRoll = new double[3];
   double[] temporaryYawPitchRoll = new double[3];
   private final YoDouble startYaw;
   private final YoDouble goalYaw;
   private final YoDouble interpolatedYaw;
   
   //used to check if the orientation error is too big
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
   
   //for feedBack in scs
   private final YoFramePose yoStartOffsetErrorPose_InWorldFrame;
   private final YoFramePose yoGoalOffsetErrorPose_InWorldFrame;
   private final YoFramePose yoInterpolatedOffset_InWorldFrame;

   private final FramePose3D startOffsetErrorPose_Translation = new FramePose3D(worldFrame);
   private final FramePose3D startOffsetErrorPose_Rotation = new FramePose3D(worldFrame);
   private final PoseReferenceFrame startOffsetErrorReferenceFrame_Translation = new PoseReferenceFrame("startOffsetErrorReferenceFrame_Translation", startOffsetErrorPose_Translation);
   private final PoseReferenceFrame startOffsetErrorReferenceFrame_Rotation= new PoseReferenceFrame("startOffsetErrorReferenceFrame_Rotation", startOffsetErrorPose_Rotation);

   private final FramePoint3D goalOffsetFramePoint_Translation = new FramePoint3D(worldFrame);
   private final FramePoint3D interpolatedOffsetFramePoint_Translation = new FramePoint3D(worldFrame);
   private final FrameQuaternion goalOffsetFrameOrientation_Rotation = new FrameQuaternion(worldFrame);
   private final FrameQuaternion interpolatedOffsetFrameOrientation_Rotation = new FrameQuaternion(worldFrame);

   private final YoFramePoint yoGoalOffsetFramePoint_Translation;
   private final YoFramePoint yoInterpolatedOffsetFramePoint_Translation;

   private final YoFrameOrientation yoGoalOffsetFrameOrientation_Rotation;
   private final YoFrameOrientation yoInterpolatedOffsetFrameOrientation_Rotation;

   public ClippedSpeedOffsetErrorInterpolator(YoVariableRegistry parentRegistry, ReferenceFrame referenceFrame, YoDouble alphaFilterBreakFrequency,
         double estimator_dt, boolean correctRotation)
   {
      this.registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.alphaFilter_BreakFrequency = alphaFilterBreakFrequency;
      this.dt = new YoDouble("dt", registry);
      this.dt.set(estimator_dt);

      this.stateEstimatorReferenceFrame = referenceFrame;

      isRotationCorrectionEnabled = new YoBoolean("isRotationCorrectionEnabled", registry);
      isRotationCorrectionEnabled.set(correctRotation);

      hasBeenCalled = new YoBoolean("hasbeenCalled", registry);
      hasBeenCalled.set(false);

      alphaFilter_AlphaValue = new YoDouble("alphaFilter_AlphaValue", registry);
      alphaFilter_AlphaValue.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(this.alphaFilter_BreakFrequency.getDoubleValue(),
            this.dt.getDoubleValue()));
      alphaFilter_PositionValue = new YoDouble("alphaFilter_PositionValue", registry);
      alphaFilter_PositionValue.set(0.0);
      alphaFilter = new AlphaFilteredYoVariable("alphaFilter", registry, alphaFilter_AlphaValue, alphaFilter_PositionValue);

      cLippedAlphaFilterValue = new YoDouble("cLippedAlphaFilterValue", registry);
      cLippedAlphaFilterValue.set(0.0);
      previousClippedAlphaFilterValue = new YoDouble("previousClippedAlphaFilterValue", registry);
      previousClippedAlphaFilterValue.set(0.0);

      maxTranslationalCorrectionVelocity = new YoDouble("maxTranslationalCorrectionVelocity", registry);
      maxTranslationalCorrectionVelocity.set(MAX_TRANSLATIONAL_CORRECTION_SPEED);
      maxRotationalCorrectionVelocity = new YoDouble("maxRotationalCorrectionVelocity", registry);
      maxRotationalCorrectionVelocity.set(MAX_ROTATIONAL_CORRECTION_SPEED);

      distanceToTravel = new YoDouble("distanceToTravel", registry);
      distanceToTravel.set(0.0);
      angleToTravel = new YoDouble("angleToTravel", registry);
      angleToTravel.set(0.0);

      translationalSpeedForGivenDistanceToTravel = new YoDouble("translationalSpeedForGivenDistanceToTravel", registry);
      rotationalSpeedForGivenAngleToTravel = new YoDouble("rotationalSpeedForGivenAngleToTravel", registry);

      temporaryTranslationAlphaClipped = new YoDouble("temporaryTranslationAlphaClipped", registry);
      temporaryRotationAlphaClipped = new YoDouble("temporaryRotationAlphaClipped", registry);

      xDeadzoneSize = new YoDouble("xDeadzoneSize", registry);
      xDeadzoneSize.set(X_DEADZONE_SIZE);
      yDeadzoneSize = new YoDouble("yDeadzoneSize", registry);
      yDeadzoneSize.set(Y_DEADZONE_SIZE);
      zDeadzoneSize = new YoDouble("zDeadzoneSize", registry);
      zDeadzoneSize.set(Z_DEADZONE_SIZE);

      goalTranslationRawX = new YoDouble("goalTranslationRawX", registry);
      goalTranslationRawY = new YoDouble("goalTranslationRawY", registry);
      goalTranslationRawZ = new YoDouble("goalTranslationRawZ", registry);

      goalTranslationWithDeadzoneX = new DeadzoneYoVariable("goalTranslationWithDeadzoneX", goalTranslationRawX, xDeadzoneSize, registry);
      goalTranslationWithDeadzoneY = new DeadzoneYoVariable("goalTranslationWithDeadzoneY", goalTranslationRawY, yDeadzoneSize, registry);
      goalTranslationWithDeadzoneZ = new DeadzoneYoVariable("goalTranslationWithDeadzoneZ", goalTranslationRawZ, zDeadzoneSize, registry);
      
      yawDeadzoneSize = new YoDouble("yawDeadzoneSize", registry);
      yawDeadzoneSize.set(Math.toRadians(YAW_DEADZONE_IN_DEGREES));
      goalYawRaw = new YoDouble("goalYawRaw", registry);
      goalYawWithDeadZone = new DeadzoneYoVariable("goalYawWithDeadZone", goalYawRaw, yawDeadzoneSize, registry);
      
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
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            alphaFilter_AlphaValue.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(alphaFilter_BreakFrequency.getDoubleValue(),
                  dt.getDoubleValue()));
         }
      });

      maximumErrorAngleInDegrees = new YoDouble("maximumErrorAngleInDegrees", registry);
      maximumErrorAngleInDegrees.set(MAXIMUM_ANGLE_ERROR_IN_DEGRESS);
      maximumErrorTranslation = new YoDouble("maximumErrorTranslation", registry);
      maximumErrorTranslation.set(MAXIMUM_TRANSLATION_ERROR);
      
      startYaw = new YoDouble("startYaw", registry);
      goalYaw = new YoDouble("goalYaw", registry);
      interpolatedYaw = new YoDouble("interpolatedYaw", registry);
    
      angleError = new YoDouble("angleError", registry);
      translationErrorX = new YoDouble("translationErrorX", registry);
      translationErrorY = new YoDouble("translationErrorY", registry);
      translationErrorZ = new YoDouble("translationErrorZ", registry);
      
   }

   public boolean checkIfErrorIsTooBig(FramePose3D correctedPelvisPoseInWorldFrame, FramePose3D iterativeClosestPointInWorldFramePose, boolean isRotationCorrectionEnabled)
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
      
      if(isRotationCorrectionEnabled && Math.abs(axisAngleForError.getAngle()) > Math.toRadians(maximumErrorAngleInDegrees.getDoubleValue()))
         return true;
      
      if(Math.abs(iterativeClosestPointTranslation.getX()) > maximumErrorTranslation.getDoubleValue())
         return true;
      if(Math.abs(iterativeClosestPointTranslation.getY()) > maximumErrorTranslation.getDoubleValue())
         return true;
      if(Math.abs(iterativeClosestPointTranslation.getZ()) > maximumErrorTranslation.getDoubleValue())
         return true;
      
      return false;
   }

   public void setInterpolatorInputs(FramePose3D startOffsetError, FramePose3D goalOffsetError, double alphaFilterPosition)
   {
      startOffsetErrorPose.setIncludingFrame(startOffsetError);
      goalOffsetErrorPose.setIncludingFrame(goalOffsetError);
      if (!isRotationCorrectionEnabled.getBooleanValue())
      {
         startOffsetErrorPose.setOrientationYawPitchRoll(0.0, 0.0, 0.0);
         goalOffsetErrorPose.setOrientationYawPitchRoll(0.0, 0.0, 0.0);
      }
      //scs feedback only
      yoStartOffsetErrorPose_InWorldFrame.set(startOffsetErrorPose);
      yoGoalOffsetErrorPose_InWorldFrame.set(goalOffsetErrorPose);

      stateEstimatorReferenceFrame.update();
      
      updateStartAndGoalOffsetErrorTranslation();
      updateStartAndGoalOffsetErrorRotation();
      
      /////////////////////

      initializeAlphaFilter(alphaFilterPosition);
      
      updateMaxTranslationAlphaVariationSpeed();
      updateMaxRotationAlphaVariationSpeed();
   }

   private void initializeAlphaFilter(double alphaFilterPosition)
   {
      alphaFilter_PositionValue.set(alphaFilterPosition);
      alphaFilter.set(0.0);
      previousClippedAlphaFilterValue.set(0.0);
      cLippedAlphaFilterValue.set(0.0);
      hasBeenCalled.set(false);
   }
   
   private void updateStartAndGoalOffsetErrorTranslation()
   {
      //Translation
      stateEstimatorReferenceFrame.getTransformToDesiredFrame(stateEstimatorTransform_Translation, worldFrame);
      stateEstimatorTransform_Translation.setRotationToZero();
      stateEstimatorPose_Translation.set(stateEstimatorTransform_Translation);
      stateEstimatorReferenceFrame_Translation.setPoseAndUpdate(stateEstimatorPose_Translation);
      
      startOffsetErrorPose.changeFrame(stateEstimatorReferenceFrame_Translation);
      updatedStartOffset_Translation.set(startOffsetErrorPose.getPosition());
      
      goalOffsetErrorPose.changeFrame(stateEstimatorReferenceFrame_Translation);
      updatedGoalOffset_Translation.set(goalOffsetErrorPose.getPosition());
      
      startOffsetTransform_Translation.setTranslationAndIdentityRotation(updatedStartOffset_Translation);
      
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
   }

   private void updateStartAndGoalOffsetErrorRotation()
   {    
      //Rotation  
      stateEstimatorReferenceFrame.getTransformToDesiredFrame(stateEstimatorTransform_Rotation, worldFrame);
      stateEstimatorTransform_Rotation.setTranslationToZero();
      stateEstimatorPose_Rotation.set(stateEstimatorTransform_Rotation);
      stateEstimatorReferenceFrame_Rotation.setPoseAndUpdate(stateEstimatorPose_Rotation);

      startOffsetErrorPose.changeFrame(stateEstimatorReferenceFrame_Rotation);
      updatedStartOffset_Rotation_quat.set(startOffsetErrorPose.getOrientation());
      startOffsetErrorPose.changeFrame(worldFrame);
      updatedStartOffset_Rotation.setIncludingFrame(startOffsetErrorPose.getOrientation());
      
      goalOffsetErrorPose.changeFrame(stateEstimatorReferenceFrame_Rotation);
      updatedGoalOffset_Rotation_quat.set(goalOffsetErrorPose.getOrientation());
      goalOffsetErrorPose.changeFrame(worldFrame);
      updatedGoalOffset_Rotation.setIncludingFrame(goalOffsetErrorPose.getOrientation());
      
      startOffsetTransform_Rotation.setRotationAndZeroTranslation(updatedStartOffset_Rotation_quat);
      
      offsetBetweenStartAndGoal_Rotation.difference(updatedStartOffset_Rotation, updatedGoalOffset_Rotation);
      offsetBetweenStartAndGoal_Rotation.getYawPitchRoll(temporaryYawPitchRoll);
      goalYawRaw.set(temporaryYawPitchRoll[0]);
      goalYawWithDeadZone.update();
      updatedGoalOffsetWithDeadZone_Rotation_quat.setYawPitchRoll(goalYawWithDeadZone.getDoubleValue(), temporaryYawPitchRoll[1], temporaryYawPitchRoll[2]);
      updatedGoalOffsetWithDeadZone_Rotation_quat.multiply(updatedStartOffset_Rotation_quat, updatedGoalOffsetWithDeadZone_Rotation_quat);
      updatedGoalOffsetWithDeadZone_Rotation.set(updatedGoalOffsetWithDeadZone_Rotation_quat);
      
      goalOffsetTransform_Rotation.setRotationAndZeroTranslation(updatedGoalOffsetWithDeadZone_Rotation_quat);
   }

   private void updateMaxTranslationAlphaVariationSpeed()
   {
      //Translation
      distanceToTravelVector.sub(updatedGoalOffsetWithDeadzone_Translation, updatedStartOffset_Translation);
      distanceToTravel.set(distanceToTravelVector.length());
      translationalSpeedForGivenDistanceToTravel.set(distanceToTravel.getDoubleValue() / dt.getDoubleValue());
   }

   private void updateMaxRotationAlphaVariationSpeed()
   {
      //Rotation
      rotationToTravel.difference(updatedGoalOffsetWithDeadZone_Rotation, updatedStartOffset_Rotation);
      axisAngletoTravel.set(rotationToTravel);
      angleToTravel.set(axisAngletoTravel.getAngle());
      rotationalSpeedForGivenAngleToTravel.set(angleToTravel.getDoubleValue() / dt.getDoubleValue());

   }
   
   public void interpolateError(FramePose3D offsetPoseToPack)
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
      cLippedAlphaFilterValue.set(MathTools.clamp(cLippedAlphaFilterValue.getDoubleValue(), 0.0, 1.0));
      
      
      previousClippedAlphaFilterValue.set(cLippedAlphaFilterValue.getDoubleValue());

      stateEstimatorReferenceFrame.getTransformToDesiredFrame(stateEstimatorTransform_Translation, worldFrame);
      stateEstimatorTransform_Translation.setRotationToZero();
      stateEstimatorReferenceFrame.getTransformToDesiredFrame(stateEstimatorTransform_Rotation, worldFrame);
      stateEstimatorTransform_Rotation.setTranslationToZero();

      //update the start and goal translations and rotations
      //localization translation times interpolated translation times localization rotation times interpolated rotation
      //the interpolated translation and rotation are those stored when a localization packet has been received
      updatedStartOffsetTransform.setIdentity();
      updatedStartOffsetTransform.multiply(startOffsetTransform_Translation);
      updatedStartOffsetTransform.multiply(stateEstimatorTransform_Translation);
      updatedStartOffsetTransform.multiply(startOffsetTransform_Rotation);
      updatedStartOffsetTransform.multiply(stateEstimatorTransform_Rotation);

      updatedStartOffsetTransform.getTranslation(updatedStartOffset_Translation);
      updatedStartOffsetTransform.getRotation(updatedStartOffset_Rotation_quat);

      updatedGoalOffsetTransform.setIdentity();
      updatedGoalOffsetTransform.multiply(goalOffsetTransform_Translation);
      updatedGoalOffsetTransform.multiply(stateEstimatorTransform_Translation);
      updatedGoalOffsetTransform.multiply(goalOffsetTransform_Rotation);
      updatedGoalOffsetTransform.multiply(stateEstimatorTransform_Rotation);

      updatedGoalOffsetTransform.getTranslation(updatedGoalOffset_Translation);
      updatedGoalOffsetTransform.getRotation(updatedGoalOffset_Rotation_quat);

      //interpolation here
      interpolatedTranslation.interpolate(updatedStartOffset_Translation, updatedGoalOffset_Translation, cLippedAlphaFilterValue.getDoubleValue());

      if (isRotationCorrectionEnabled.getBooleanValue())
      {
         stateEstimatorTransform_Rotation.getRotationYawPitchRoll(stateEstimatorYawPitchRoll);
         
         updatedStartOffset_Rotation_quat.getYawPitchRoll(temporaryYawPitchRoll);
         startYaw.set(temporaryYawPitchRoll[0]);
         
         updatedGoalOffset_Rotation_quat.getYawPitchRoll(temporaryYawPitchRoll);
         goalYaw.set(temporaryYawPitchRoll[0]);
         
         interpolatedYaw.set((1 - cLippedAlphaFilterValue.getDoubleValue()) * startYaw.getDoubleValue() + cLippedAlphaFilterValue.getDoubleValue() * goalYaw.getDoubleValue());
         interpolatedRotation.setYawPitchRoll(interpolatedYaw.getDoubleValue(), stateEstimatorYawPitchRoll[1], stateEstimatorYawPitchRoll[2]);
      }
      else
      {
         stateEstimatorTransform_Rotation.getRotation(interpolatedRotation);
      }

      offsetPoseToPack.set(interpolatedTranslation, interpolatedRotation);

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

   public void setDeadZoneSizes(double xDeadzoneSize, double yDeadzoneSize, double zDeadzoneSize, double yawDeadzoneSize)
   {
      this.xDeadzoneSize.set(xDeadzoneSize);
      this.yDeadzoneSize.set(yDeadzoneSize);
      this.zDeadzoneSize.set(zDeadzoneSize);
      this.yawDeadzoneSize.set(yawDeadzoneSize);
   }
}
