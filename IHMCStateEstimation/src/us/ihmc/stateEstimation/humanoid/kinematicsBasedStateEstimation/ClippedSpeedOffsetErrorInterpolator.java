package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

public class ClippedSpeedOffsetErrorInterpolator
{
   private static final double MAX_TRANSLATIONAL_CORRECTION_SPEED = 0.05;
   private static final double MAX_ROTATIONAL_CORRECTION_SPEED = 0.05;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final BooleanYoVariable isRotationCorrectionEnabled;

   private final YoVariableRegistry registry;

   private final BooleanYoVariable hasBeenCalled;

   private final ReferenceFrame referenceFrame;

   private final DoubleYoVariable alphaFilterBreakFrequency;
   private final DoubleYoVariable dt;

   private final Vector3d startTranslation = new Vector3d();
   private final Vector3d interpolatedTranslation = new Vector3d();
   private final Vector3d goalTranslation = new Vector3d();

   private final Quat4d startRotation = new Quat4d();
   private final Quat4d interpolatedRotation = new Quat4d();
   private final Quat4d goalRotation = new Quat4d();

   private final AlphaFilteredYoVariable alphaFilter;
   private final DoubleYoVariable alphaFilter_AlphaValue;
   private final DoubleYoVariable alphaFilter_PositionValue;
   private final DoubleYoVariable cLippedAlphaFilterValue;
   private final DoubleYoVariable previousClippedAlphaFilterValue;

   private final DoubleYoVariable maxTranslationalCorrectionVelocity;
   private final DoubleYoVariable maxRotationalCorrectionVelocity;

   private final FramePose startOffsetErrorPose = new FramePose();
   private final PoseReferenceFrame startOffsetErrorPoseReferenceFrame = new PoseReferenceFrame("startOffsetReferenceFrame", startOffsetErrorPose);
   private final FramePose goalOffsetErrorPose = new FramePose();
   private final RigidBodyTransform errorBetweenStartAndGoalOffsetErrorTransform = new RigidBodyTransform();
   private final Vector3d distanceToTravelVector = new Vector3d();
   private final DoubleYoVariable distanceToTravel;
   private final AxisAngle4d angleToTravelAxis4d = new AxisAngle4d();
   private final DoubleYoVariable angleToTravel;

   private final DoubleYoVariable translationalSpeedForGivenDistanceToTravel;
   private final DoubleYoVariable rotationalSpeedForGivenAngleToTravel;

   private final DoubleYoVariable temporaryTranslationAlphaClipped;
   private final DoubleYoVariable temporaryRotationAlphaClipped;

   //for feedBack in scs
   private final YoFramePose yoStartOffsetErrorPose;
   private final YoFramePose yoGoalOffsetErrorPose;
   private final YoFramePose yoInterpolatedOffset;

   public ClippedSpeedOffsetErrorInterpolator(YoVariableRegistry parentRegistry, ReferenceFrame referenceFrame, DoubleYoVariable alphaFilterBreakFrequency,
         double dt, boolean correctRotation)
   {
      this.registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.alphaFilterBreakFrequency = alphaFilterBreakFrequency;
      this.dt = new DoubleYoVariable("dt", registry);
      this.dt.set(dt);

      this.referenceFrame = referenceFrame;

      isRotationCorrectionEnabled = new BooleanYoVariable("isRotationCorrectionEnabled", registry);
      isRotationCorrectionEnabled.set(correctRotation);

      hasBeenCalled = new BooleanYoVariable("hasbeenCalled", registry);
      hasBeenCalled.set(false);

      alphaFilter_AlphaValue = new DoubleYoVariable("alphaFilter_AlphaValue", registry);
      alphaFilter_AlphaValue.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(alphaFilterBreakFrequency.getDoubleValue(),
            this.dt.getDoubleValue()));
      alphaFilter_PositionValue = new DoubleYoVariable("alphaFilter_PositionValue", registry);
      alphaFilter_PositionValue.set(0.0);
      alphaFilter = new AlphaFilteredYoVariable("alphaFilter", registry, alphaFilter_AlphaValue, alphaFilter_PositionValue);

      cLippedAlphaFilterValue = new DoubleYoVariable("cLippedAlphaFilterValue", registry);
      cLippedAlphaFilterValue.set(0.0);
      previousClippedAlphaFilterValue = new DoubleYoVariable("previousClippedAlphaFilterValue", registry);
      previousClippedAlphaFilterValue.set(0.0);

      maxTranslationalCorrectionVelocity = new DoubleYoVariable("maxTranslationalCorrectionVelocity", parentRegistry);
      maxTranslationalCorrectionVelocity.set(MAX_TRANSLATIONAL_CORRECTION_SPEED * this.dt.getDoubleValue());
      maxRotationalCorrectionVelocity = new DoubleYoVariable("maxRotationalCorrectionVelocity", parentRegistry);
      maxRotationalCorrectionVelocity.set(MAX_ROTATIONAL_CORRECTION_SPEED * this.dt.getDoubleValue());

      distanceToTravel = new DoubleYoVariable("distanceToTravel", registry);
      distanceToTravel.set(0.0);
      angleToTravel = new DoubleYoVariable("angleToTravel", registry);
      angleToTravel.set(0.0);

      translationalSpeedForGivenDistanceToTravel = new DoubleYoVariable("translationalSpeedForGivenDistanceToTravel", registry);
      rotationalSpeedForGivenAngleToTravel = new DoubleYoVariable("rotationalSpeedForGivenAngleToTravel", registry);

      temporaryTranslationAlphaClipped = new DoubleYoVariable("temporaryTranslationAlphaClipped", registry);
      temporaryRotationAlphaClipped = new DoubleYoVariable("temporaryRotationAlphaClipped", registry);

      // for feedback in SCS
      yoStartOffsetErrorPose = new YoFramePose("yoStartOffsetErrorPose", this.referenceFrame, registry);
      yoGoalOffsetErrorPose = new YoFramePose("yoGoalOffsetErrorPose", this.referenceFrame, registry);
      yoInterpolatedOffset = new YoFramePose("yoInterpolatedOffset", this.referenceFrame, registry);
   }

   public void setInterpolatorInputs(FramePose startOffsetError, FramePose goalOffsetError, double alphaFilterPosition)
   {
      startOffsetError.checkReferenceFrameMatch(referenceFrame);
      goalOffsetError.checkReferenceFrameMatch(referenceFrame);

      this.startOffsetErrorPose.setPoseIncludingFrame(startOffsetError);
      this.goalOffsetErrorPose.setPoseIncludingFrame(goalOffsetError);
      if (!isRotationCorrectionEnabled.getBooleanValue())
      {
         this.startOffsetErrorPose.setOrientation(0.0, 0.0, 0.0);
         this.goalOffsetErrorPose.setOrientation(0.0, 0.0, 0.0);
      }

      //scs feedback only
      yoStartOffsetErrorPose.set(startOffsetError);
      yoGoalOffsetErrorPose.set(goalOffsetError);
      ///////////

      startOffsetError.getPose(startTranslation, startRotation);
      goalOffsetError.getPose(goalTranslation, goalRotation);

      if (!isRotationCorrectionEnabled.getBooleanValue())
      {
         startRotation.set(0.0, 0.0, 0.0, 1.0);
         goalRotation.set(0.0, 0.0, 0.0, 1.0);
      }
      
      alphaFilter_PositionValue.set(alphaFilterPosition);
      alphaFilter.set(0.0);
      previousClippedAlphaFilterValue.set(0.0);
      cLippedAlphaFilterValue.set(0.0);

      updateMaxAlphaVariationSpeed();

      hasBeenCalled.set(false);
   }

   private void updateMaxAlphaVariationSpeed()
   {
      startOffsetErrorPose.changeFrame(worldFrame);
      startOffsetErrorPoseReferenceFrame.setPoseAndUpdate(startOffsetErrorPose);

      goalOffsetErrorPose.changeFrame(startOffsetErrorPoseReferenceFrame);
      goalOffsetErrorPose.getPose(errorBetweenStartAndGoalOffsetErrorTransform);

      errorBetweenStartAndGoalOffsetErrorTransform.getTranslation(distanceToTravelVector);
      distanceToTravel.set(distanceToTravelVector.length());
      translationalSpeedForGivenDistanceToTravel.set(distanceToTravel.getDoubleValue() / dt.getDoubleValue());

      errorBetweenStartAndGoalOffsetErrorTransform.getRotation(angleToTravelAxis4d);
      angleToTravel.set(angleToTravelAxis4d.getAngle());
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
            + (maxTranslationalCorrectionVelocity.getDoubleValue() / distanceToTravel.getDoubleValue()))
      {
         temporaryTranslationAlphaClipped.set(alphaFilter.getDoubleValue());
      }
      else
      {
         temporaryTranslationAlphaClipped.set(previousClippedAlphaFilterValue.getDoubleValue()
               + (maxTranslationalCorrectionVelocity.getDoubleValue() / distanceToTravel.getDoubleValue()));
      }

      //rotation
      if (rotationalSpeedForGivenAngleToTravel.getDoubleValue() * alphaFilter.getDoubleValue() <= rotationalSpeedForGivenAngleToTravel.getDoubleValue()
            * previousClippedAlphaFilterValue.getDoubleValue() + (maxRotationalCorrectionVelocity.getDoubleValue() / angleToTravel.getDoubleValue()))
      {
         temporaryRotationAlphaClipped.set(alphaFilter.getDoubleValue());
      }
      else
      {
         temporaryRotationAlphaClipped.set(previousClippedAlphaFilterValue.getDoubleValue()
               + (maxRotationalCorrectionVelocity.getDoubleValue() / angleToTravel.getDoubleValue()));
      }

      //chose the smaller alpha (so the slower correction) between translation and rotation
      cLippedAlphaFilterValue.set(Math.min(temporaryTranslationAlphaClipped.getDoubleValue(), temporaryRotationAlphaClipped.getDoubleValue()));
      previousClippedAlphaFilterValue.set(cLippedAlphaFilterValue.getDoubleValue());

      interpolatedTranslation.interpolate(startTranslation, goalTranslation, cLippedAlphaFilterValue.getDoubleValue());
      interpolatedRotation.interpolate(startRotation, goalRotation, cLippedAlphaFilterValue.getDoubleValue());

      offsetPoseToPack.setPose(interpolatedTranslation, interpolatedRotation);

      //scs feedback only
      yoInterpolatedOffset.set(offsetPoseToPack);
   }
}
