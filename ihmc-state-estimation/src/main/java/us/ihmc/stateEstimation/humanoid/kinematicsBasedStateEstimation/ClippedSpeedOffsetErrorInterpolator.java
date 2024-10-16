package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.filters.AlphaFilterTools;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.filters.DeadbandedYoVariable;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class ClippedSpeedOffsetErrorInterpolator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoBoolean isRotationCorrectionEnabled;

   private final YoRegistry registry;

   private final YoBoolean hasBeenCalled;

   private final YoDouble dt;

   ////////////////////////////////////////////
   private final FramePose3D stateEstimatorPose_Translation = new FramePose3D(worldFrame);
   private final PoseReferenceFrame stateEstimatorReferenceFrame_Translation = new PoseReferenceFrame("stateEstimatorReferenceFrame_Translation",
                                                                                                      stateEstimatorPose_Translation);
   private final FramePose3D stateEstimatorPose_Rotation = new FramePose3D(worldFrame);
   private final PoseReferenceFrame stateEstimatorReferenceFrame_Rotation = new PoseReferenceFrame("stateEstimatorReferenceFrame_Rotation",
                                                                                                   stateEstimatorPose_Rotation);

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

   private final YoDouble alphaFilterBreakFrequency;

   private final AlphaFilteredYoVariable alphaFilter;
   private final YoDouble alphaFilter_AlphaValue;
   private final YoDouble alphaFilter_PositionValue;
   private final YoDouble cLippedAlphaFilterValue;

   private final YoDouble maxTranslationalCorrectionVelocity;
   private final YoDouble maxRotationalCorrectionVelocity;

   private final YoDouble maximumAlphaFilterChangeTranslation;
   private final YoDouble maximumAlphaFilterChangeRotation;
   private final YoDouble maximumAlphaFilterChange;

   private final Vector3D distanceToTravelVector = new Vector3D();
   private final YoDouble distanceToTravel;

   private final YoDouble angleToTravel;

   private final ReferenceFrame stateEstimatorReferenceFrame;
   private final RigidBodyTransform stateEstimatorTransform_Translation = new RigidBodyTransform();
   private final RigidBodyTransform stateEstimatorTransform_Rotation = new RigidBodyTransform();

   //Deadzone translation variables
   private final YoDouble xDeadzoneSize;
   private final YoDouble yDeadzoneSize;
   private final YoDouble zDeadzoneSize;
   private final DeadbandedYoVariable goalTranslationWithDeadzoneX;
   private final DeadbandedYoVariable goalTranslationWithDeadzoneY;
   private final DeadbandedYoVariable goalTranslationWithDeadzoneZ;
   private final YoDouble goalTranslationRawX;
   private final YoDouble goalTranslationRawY;
   private final YoDouble goalTranslationRawZ;
   private final Vector3D offsetBetweenStartAndGoalVector_Translation = new Vector3D();
   private final Vector3D updatedGoalOffsetWithDeadzone_Translation = new Vector3D();

   //Deadzone rotation Variables
   private final YoDouble yawDeadzoneSize;
   private final DeadbandedYoVariable goalYawWithDeadZone;
   private final YoDouble goalYawRaw;
   private final FrameQuaternion offsetBetweenStartAndGoal_Rotation = new FrameQuaternion(worldFrame);
   private final FrameQuaternion updatedGoalOffsetWithDeadZone_Rotation = new FrameQuaternion(worldFrame);
   private final Quaternion updatedGoalOffsetWithDeadZone_Rotation_quat = new Quaternion();

   YawPitchRoll stateEstimatorYawPitchRoll = new YawPitchRoll();
   YawPitchRoll temporaryYawPitchRoll = new YawPitchRoll();
   private final YoDouble startYaw;
   private final YoDouble goalYaw;
   private final YoDouble interpolatedYaw;

   //for feedBack in scs
   private final YoFramePoseUsingYawPitchRoll yoStartOffsetErrorPose_InWorldFrame;
   private final YoFramePoseUsingYawPitchRoll yoGoalOffsetErrorPose_InWorldFrame;
   private final YoFramePoseUsingYawPitchRoll yoInterpolatedOffset_InWorldFrame;

   private final FramePose3D startOffsetErrorPose_Translation = new FramePose3D(worldFrame);
   private final FramePose3D startOffsetErrorPose_Rotation = new FramePose3D(worldFrame);
   private final PoseReferenceFrame startOffsetErrorReferenceFrame_Translation = new PoseReferenceFrame("startOffsetErrorReferenceFrame_Translation",
                                                                                                        startOffsetErrorPose_Translation);
   private final PoseReferenceFrame startOffsetErrorReferenceFrame_Rotation = new PoseReferenceFrame("startOffsetErrorReferenceFrame_Rotation",
                                                                                                     startOffsetErrorPose_Rotation);

   private final FramePoint3D goalOffsetFramePoint_Translation = new FramePoint3D(worldFrame);
   private final FramePoint3D interpolatedOffsetFramePoint_Translation = new FramePoint3D(worldFrame);
   private final FrameQuaternion goalOffsetFrameOrientation_Rotation = new FrameQuaternion(worldFrame);
   private final FrameQuaternion interpolatedOffsetFrameOrientation_Rotation = new FrameQuaternion(worldFrame);

   private final YoFramePoint3D yoGoalOffsetFramePoint_Translation;
   private final YoFramePoint3D yoInterpolatedOffsetFramePoint_Translation;

   private final YoFrameYawPitchRoll yoGoalOffsetFrameOrientation_Rotation;
   private final YoFrameYawPitchRoll yoInterpolatedOffsetFrameOrientation_Rotation;

   public ClippedSpeedOffsetErrorInterpolator(YoRegistry parentRegistry, ReferenceFrame referenceFrame, double estimator_dt)
   {
      this(parentRegistry, referenceFrame, estimator_dt, new ClippedSpeedOffsetErrorInterpolatorParameters());
   }

   public ClippedSpeedOffsetErrorInterpolator(YoRegistry parentRegistry, ReferenceFrame referenceFrame, double estimator_dt, ClippedSpeedOffsetErrorInterpolatorParameters parameters)
   {
      this.registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.alphaFilterBreakFrequency = new YoDouble("alphaFilterBreakFrequency", registry);
      this.alphaFilterBreakFrequency.set(parameters.getBreakFrequency());

      this.dt = new YoDouble("dt", registry);
      this.dt.set(estimator_dt);

      this.stateEstimatorReferenceFrame = referenceFrame;

      isRotationCorrectionEnabled = new YoBoolean("isRotationCorrectionEnabled", registry);
      isRotationCorrectionEnabled.set(parameters.getIsRotationCorrectionEnabled());

      hasBeenCalled = new YoBoolean("hasbeenCalled", registry);
      hasBeenCalled.set(false);

      alphaFilter_AlphaValue = new YoDouble("alphaFilter_AlphaValue", registry);
      alphaFilter_AlphaValue.set(AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(this.alphaFilterBreakFrequency.getValue(),
                                                                                          this.dt.getValue()));
      alphaFilter_PositionValue = new YoDouble("alphaFilter_PositionValue", registry);
      alphaFilter_PositionValue.set(0.0);
      alphaFilter = new AlphaFilteredYoVariable("alphaFilter", registry, alphaFilter_AlphaValue, alphaFilter_PositionValue);

      cLippedAlphaFilterValue = new YoDouble("cLippedAlphaFilterValue", registry);

      maxTranslationalCorrectionVelocity = new YoDouble("maxTranslationalCorrectionVelocity", registry);
      maxTranslationalCorrectionVelocity.set(parameters.getMaxTranslationalCorrectionSpeed());
      maxRotationalCorrectionVelocity = new YoDouble("maxRotationalCorrectionVelocity", registry);
      maxRotationalCorrectionVelocity.set(parameters.getMaxRotationalCorrectionSpeed());

      maximumAlphaFilterChangeTranslation = new YoDouble("maximumAlphaFilterChangeTranslation", registry);
      maximumAlphaFilterChangeRotation = new YoDouble("maximumAlphaFilterChangeRotation", registry);
      maximumAlphaFilterChange = new YoDouble("maximumAlphaFilterChange", registry);

      distanceToTravel = new YoDouble("distanceToTravel", registry);
      distanceToTravel.set(0.0);
      angleToTravel = new YoDouble("angleToTravel", registry);
      angleToTravel.set(0.0);

      xDeadzoneSize = new YoDouble("xDeadzoneSize", registry);
      xDeadzoneSize.set(parameters.getXDeadzoneSize());
      yDeadzoneSize = new YoDouble("yDeadzoneSize", registry);
      yDeadzoneSize.set(parameters.getYDeadzoneSize());
      zDeadzoneSize = new YoDouble("zDeadzoneSize", registry);
      zDeadzoneSize.set(parameters.getZDeadzoneSize());

      goalTranslationRawX = new YoDouble("goalTranslationRawX", registry);
      goalTranslationRawY = new YoDouble("goalTranslationRawY", registry);
      goalTranslationRawZ = new YoDouble("goalTranslationRawZ", registry);

      goalTranslationWithDeadzoneX = new DeadbandedYoVariable("goalTranslationWithDeadzoneX", goalTranslationRawX, xDeadzoneSize, registry);
      goalTranslationWithDeadzoneY = new DeadbandedYoVariable("goalTranslationWithDeadzoneY", goalTranslationRawY, yDeadzoneSize, registry);
      goalTranslationWithDeadzoneZ = new DeadbandedYoVariable("goalTranslationWithDeadzoneZ", goalTranslationRawZ, zDeadzoneSize, registry);

      yawDeadzoneSize = new YoDouble("yawDeadzoneSize", registry);
      yawDeadzoneSize.set(Math.toRadians(parameters.getYawDeadzoneInDegrees()));
      goalYawRaw = new YoDouble("goalYawRaw", registry);
      goalYawWithDeadZone = new DeadbandedYoVariable("goalYawWithDeadZone", goalYawRaw, yawDeadzoneSize, registry);

      // for feedback in SCS
      yoStartOffsetErrorPose_InWorldFrame = new YoFramePoseUsingYawPitchRoll("yoStartOffsetErrorPose_InWorldFrame", worldFrame, registry);
      yoGoalOffsetErrorPose_InWorldFrame = new YoFramePoseUsingYawPitchRoll("yoGoalOffsetErrorPose_InWorldFrame", worldFrame, registry);
      yoInterpolatedOffset_InWorldFrame = new YoFramePoseUsingYawPitchRoll("yoInterpolatedOffset_InWorldFrame", worldFrame, registry);

      yoGoalOffsetFramePoint_Translation = new YoFramePoint3D("yoGoalOffsetFramePoint_Translation", startOffsetErrorReferenceFrame_Translation, registry);
      yoInterpolatedOffsetFramePoint_Translation = new YoFramePoint3D("yoInterpolatedOffsetFramePoint_Translation",
                                                                      startOffsetErrorReferenceFrame_Translation,
                                                                      registry);

      yoGoalOffsetFrameOrientation_Rotation = new YoFrameYawPitchRoll("yoGoalOffsetFrameOrientation_Rotation",
                                                                      startOffsetErrorReferenceFrame_Rotation,
                                                                      registry);
      yoInterpolatedOffsetFrameOrientation_Rotation = new YoFrameYawPitchRoll("yoInterpolatedOffsetFrameOrientation_Rotation",
                                                                              startOffsetErrorReferenceFrame_Rotation,
                                                                              registry);

      this.alphaFilterBreakFrequency.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            alphaFilter_AlphaValue.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(alphaFilterBreakFrequency.getValue(), dt.getValue()));
         }
      });

      startYaw = new YoDouble("startYaw", registry);
      goalYaw = new YoDouble("goalYaw", registry);
      interpolatedYaw = new YoDouble("interpolatedYaw", registry);
   }

   public void setInterpolatorInputs(FramePose3D startOffsetError, FramePose3D goalOffsetError, double alphaFilterPosition)
   {
      startOffsetErrorPose.setIncludingFrame(startOffsetError);
      goalOffsetErrorPose.setIncludingFrame(goalOffsetError);
      if (!isRotationCorrectionEnabled.getBooleanValue())
      {
         startOffsetErrorPose.getOrientation().setYawPitchRoll(0.0, 0.0, 0.0);
         goalOffsetErrorPose.getOrientation().setYawPitchRoll(0.0, 0.0, 0.0);
//         goalOffsetErrorPose.set(startOffsetError);
      }
      //scs feedback only
      yoStartOffsetErrorPose_InWorldFrame.set(startOffsetErrorPose);
      yoGoalOffsetErrorPose_InWorldFrame.set(goalOffsetErrorPose);

      stateEstimatorReferenceFrame.update();

      updateStartAndGoalOffsetErrorTranslation();
      updateStartAndGoalOffsetErrorRotation();

      /////////////////////

      initializeAlphaFilter(alphaFilterPosition);
   }

   private void initializeAlphaFilter(double alphaFilterPosition)
   {
      alphaFilter_PositionValue.set(alphaFilterPosition);
      alphaFilter.set(0.0);
      hasBeenCalled.set(false);
   }

   private void updateStartAndGoalOffsetErrorTranslation()
   {
      //Translation
      stateEstimatorReferenceFrame.getTransformToDesiredFrame(stateEstimatorTransform_Translation, worldFrame);
      stateEstimatorTransform_Translation.getRotation().setToZero();
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

      updatedGoalOffsetWithDeadzone_Translation.setX(updatedStartOffset_Translation.getX() + goalTranslationWithDeadzoneX.getValue());
      updatedGoalOffsetWithDeadzone_Translation.setY(updatedStartOffset_Translation.getY() + goalTranslationWithDeadzoneY.getValue());
      updatedGoalOffsetWithDeadzone_Translation.setZ(updatedStartOffset_Translation.getZ() + goalTranslationWithDeadzoneZ.getValue());

      goalOffsetTransform_Translation.setTranslationAndIdentityRotation(updatedGoalOffsetWithDeadzone_Translation);
   }

   private void updateStartAndGoalOffsetErrorRotation()
   {
      //Rotation  
      stateEstimatorReferenceFrame.getTransformToDesiredFrame(stateEstimatorTransform_Rotation, worldFrame);
      stateEstimatorTransform_Rotation.getTranslation().setToZero();
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
      temporaryYawPitchRoll.set(offsetBetweenStartAndGoal_Rotation);
      goalYawRaw.set(temporaryYawPitchRoll.getYaw());
      goalYawWithDeadZone.update();
      updatedGoalOffsetWithDeadZone_Rotation_quat.setYawPitchRoll(goalYawWithDeadZone.getValue(), temporaryYawPitchRoll.getPitch(), temporaryYawPitchRoll.getRoll());
      updatedGoalOffsetWithDeadZone_Rotation_quat.multiply(updatedStartOffset_Rotation_quat, updatedGoalOffsetWithDeadZone_Rotation_quat);
      updatedGoalOffsetWithDeadZone_Rotation.set(updatedGoalOffsetWithDeadZone_Rotation_quat);

      goalOffsetTransform_Rotation.setRotationAndZeroTranslation(updatedGoalOffsetWithDeadZone_Rotation_quat);
   }

   public void interpolateError(FramePose3D offsetPoseToPack)
   {
      double alphaFilterBeforeUpdate = alphaFilter.getValue();

      if (!hasBeenCalled.getBooleanValue())
      {
         alphaFilter.update(0.0);
         hasBeenCalled.set(true);
      }
      else
      {
         alphaFilter.update();
      }

      stateEstimatorReferenceFrame.getTransformToDesiredFrame(stateEstimatorTransform_Translation, worldFrame);
      stateEstimatorTransform_Translation.getRotation().setToZero();
      stateEstimatorReferenceFrame.getTransformToDesiredFrame(stateEstimatorTransform_Rotation, worldFrame);
      stateEstimatorTransform_Rotation.getTranslation().setToZero();

      //update the start and goal translations and rotations
      //localization translation times interpolated translation times localization rotation times interpolated rotation
      //the interpolated translation and rotation are those stored when a localization packet has been received
      updatedStartOffsetTransform.setIdentity();
      updatedStartOffsetTransform.multiply(startOffsetTransform_Translation);
      updatedStartOffsetTransform.multiply(stateEstimatorTransform_Translation);
      updatedStartOffsetTransform.multiply(startOffsetTransform_Rotation);
      updatedStartOffsetTransform.multiply(stateEstimatorTransform_Rotation);

      updatedStartOffset_Translation.set(updatedStartOffsetTransform.getTranslation());
      updatedStartOffset_Rotation_quat.set(updatedStartOffsetTransform.getRotation());

      updatedGoalOffsetTransform.setIdentity();
      updatedGoalOffsetTransform.multiply(goalOffsetTransform_Translation);
      updatedGoalOffsetTransform.multiply(stateEstimatorTransform_Translation);
      updatedGoalOffsetTransform.multiply(goalOffsetTransform_Rotation);
      updatedGoalOffsetTransform.multiply(stateEstimatorTransform_Rotation);

      updatedGoalOffset_Translation.set(updatedGoalOffsetTransform.getTranslation());
      updatedGoalOffset_Rotation_quat.set(updatedGoalOffsetTransform.getRotation());

      //interpolation here

      distanceToTravelVector.sub(updatedGoalOffset_Translation, updatedStartOffset_Translation);
      distanceToTravel.set(distanceToTravelVector.length());

      if (isRotationCorrectionEnabled.getBooleanValue())
      {
         stateEstimatorYawPitchRoll.set(stateEstimatorTransform_Rotation.getRotation());

         temporaryYawPitchRoll.set(updatedStartOffset_Rotation_quat);
         startYaw.set(temporaryYawPitchRoll.getYaw());

         temporaryYawPitchRoll.set(updatedGoalOffset_Rotation_quat);
         goalYaw.set(temporaryYawPitchRoll.getYaw());

         angleToTravel.set(Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(goalYaw.getValue(), startYaw.getValue())));
      }
      else
      {
         angleToTravel.set(0.0);
         interpolatedRotation.set(stateEstimatorTransform_Rotation.getRotation());
      }

      maximumAlphaFilterChangeTranslation.set(1.0);
      maximumAlphaFilterChangeRotation.set(1.0);

      if (distanceToTravel.getValue() > 1e-3)
      {
         maximumAlphaFilterChangeTranslation.set(maxTranslationalCorrectionVelocity.getValue() * dt.getValue() / distanceToTravel.getValue());
      }

      if (angleToTravel.getValue() > 1e-3)
      {
         maximumAlphaFilterChangeRotation.set(maxRotationalCorrectionVelocity.getValue() * dt.getValue() / angleToTravel.getValue());
      }

      maximumAlphaFilterChange.set(Math.min(maximumAlphaFilterChangeTranslation.getValue(), maximumAlphaFilterChangeRotation.getValue()));
      double alphaFilterChange = alphaFilter.getValue() - alphaFilterBeforeUpdate;

      if (alphaFilterChange > maximumAlphaFilterChange.getValue())
      {
         alphaFilter.set(alphaFilterBeforeUpdate + maximumAlphaFilterChange.getValue());
      }

      cLippedAlphaFilterValue.set(MathTools.clamp(alphaFilter.getValue(), 0.0, 1.0));

      interpolatedTranslation.interpolate(updatedStartOffset_Translation, updatedGoalOffset_Translation, cLippedAlphaFilterValue.getValue());

      if (isRotationCorrectionEnabled.getBooleanValue())
      {
         interpolatedYaw.set(AngleTools.interpolateAngle(startYaw.getValue(), goalYaw.getValue(), cLippedAlphaFilterValue.getValue()));
         interpolatedRotation.setYawPitchRoll(interpolatedYaw.getValue(), stateEstimatorYawPitchRoll.getPitch(), stateEstimatorYawPitchRoll.getRoll());
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
      startOffsetErrorPose_Translation.getPosition().set(updatedStartOffset_Translation);
      startOffsetErrorReferenceFrame_Translation.setPoseAndUpdate(startOffsetErrorPose_Translation);

      startOffsetErrorPose_Rotation.setToZero(worldFrame);
      startOffsetErrorPose_Rotation.getOrientation().set(updatedStartOffset_Rotation);
      startOffsetErrorReferenceFrame_Rotation.setPoseAndUpdate(startOffsetErrorPose_Rotation);

      goalOffsetFramePoint_Translation.changeFrame(worldFrame);
      goalOffsetFramePoint_Translation.set(updatedGoalOffset_Translation);
      goalOffsetFramePoint_Translation.changeFrame(startOffsetErrorReferenceFrame_Translation);
      yoGoalOffsetFramePoint_Translation.setMatchingFrame(goalOffsetFramePoint_Translation);

      interpolatedOffsetFramePoint_Translation.changeFrame(worldFrame);
      interpolatedOffsetFramePoint_Translation.set(interpolatedTranslation);
      interpolatedOffsetFramePoint_Translation.changeFrame(startOffsetErrorReferenceFrame_Translation);
      yoInterpolatedOffsetFramePoint_Translation.setMatchingFrame(interpolatedOffsetFramePoint_Translation);

      goalOffsetFrameOrientation_Rotation.changeFrame(worldFrame);
      goalOffsetFrameOrientation_Rotation.set(updatedGoalOffset_Rotation);
      goalOffsetFrameOrientation_Rotation.changeFrame(startOffsetErrorReferenceFrame_Rotation);
      yoGoalOffsetFrameOrientation_Rotation.setMatchingFrame(goalOffsetFrameOrientation_Rotation);

      interpolatedOffsetFrameOrientation_Rotation.changeFrame(worldFrame);
      interpolatedOffsetFrameOrientation_Rotation.set(interpolatedRotation);
      interpolatedOffsetFrameOrientation_Rotation.changeFrame(startOffsetErrorReferenceFrame_Rotation);
      yoInterpolatedOffsetFrameOrientation_Rotation.setMatchingFrame(interpolatedOffsetFrameOrientation_Rotation);
   }

   public void setDeadZoneSizes(double xDeadzoneSize, double yDeadzoneSize, double zDeadzoneSize, double yawDeadzoneSize)
   {
      this.xDeadzoneSize.set(xDeadzoneSize);
      this.yDeadzoneSize.set(yDeadzoneSize);
      this.zDeadzoneSize.set(zDeadzoneSize);
      this.yawDeadzoneSize.set(yawDeadzoneSize);
   }

   public void setMaximumTranslationAndRotationVelocity(double maxTranslationalCorrectionVelocity, double maxRotationalCorrectionVelocity)
   {
      this.maxTranslationalCorrectionVelocity.set(maxTranslationalCorrectionVelocity);
      this.maxRotationalCorrectionVelocity.set(maxRotationalCorrectionVelocity);
   }

   public void setAlphaFilterBreakFrequency(double breakFrequency)
   {
      this.alphaFilterBreakFrequency.set(breakFrequency);
      alphaFilter_AlphaValue.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(this.alphaFilterBreakFrequency.getValue(),
                                                                                                 this.dt.getValue()));
   }

   public void setIsRotationCorrectionEnabled(boolean isRotationCorrectionEnabled)
   {
      this.isRotationCorrectionEnabled.set(isRotationCorrectionEnabled);
   }

   public boolean isWithinDeadband(Tuple3DReadOnly translation, Orientation3DReadOnly rotation)
   {
      boolean withinXDeadband = Math.abs(translation.getX()) < xDeadzoneSize.getDoubleValue();
      boolean withinYDeadBand = Math.abs(translation.getY()) < yDeadzoneSize.getDoubleValue();
      boolean withinZDeadband = Math.abs(translation.getZ()) < zDeadzoneSize.getDoubleValue();
      boolean withinYawDeadband = Math.abs(rotation.getYaw()) < yawDeadzoneSize.getDoubleValue();
      return (withinXDeadband && withinYDeadBand && withinZDeadband && withinYawDeadband);
   }
}
