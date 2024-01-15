package us.ihmc.perception;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;

public class IterativeClosestPointObjectTrack
{
   private final FramePose3D objectPose = new FramePose3D();
   private final FrameVector3D objectLinearVelocity = new FrameVector3D();
   private final FrameVector3D objectAngularVelocity = new FrameVector3D();

   private double mostRecentMeasurementTime;

   private final IterativeClosestPointParameters icpParameters;

   public IterativeClosestPointObjectTrack(IterativeClosestPointParameters icpParameters)
   {
      this.icpParameters = icpParameters;
   }

   public void setObjectPose(Pose3DReadOnly objectPose)
   {
      this.objectPose.set(objectPose);
      objectLinearVelocity.setToZero();
      objectAngularVelocity.setToZero();

      mostRecentMeasurementTime = Conversions.nanosecondsToSeconds(System.nanoTime());
   }

   public void updateWithMeasurement(Pose3DReadOnly measuredObjectPose)
   {
      double oldTime = mostRecentMeasurementTime;
      mostRecentMeasurementTime = Conversions.nanosecondsToSeconds(System.nanoTime());

      double timeDelta = mostRecentMeasurementTime - oldTime;

      // TODO should we decay the current velocity?
      FramePose3DReadOnly posePrediction = predictObjectPoseCurrentVelocities(timeDelta);

      // If the error between the prediction and the measurement is too great, reset to the measurement.
      boolean aboveTranslationThreshold = posePrediction.getPosition().distanceSquared(measuredObjectPose.getPosition()) > MathTools.square(icpParameters.getTrackPredictionTranslationErrorToReset());
      boolean aboveRotationThreshold = Math.abs(posePrediction.getOrientation().distance(measuredObjectPose.getOrientation())) > icpParameters.getTrackPredictionOrientationErrorToReset();
      if (aboveTranslationThreshold || aboveRotationThreshold)
      {
         objectPose.set(measuredObjectPose);
         objectLinearVelocity.setToZero();
         objectAngularVelocity.setToZero();
      }
      else
      {
         FramePose3DReadOnly fusedPose = fuseMeasuredAndPredictedPose(measuredObjectPose, posePrediction, timeDelta);

         FrameVector3DReadOnly linearVelocityFromMeasurement = computeLinearVelocity(fusedPose, objectPose, timeDelta);
         FrameVector3DReadOnly angularVelocityFromMeasurement = computeAngularVelocity(fusedPose, objectPose, timeDelta);

         // update the velocity measurements using the fusing frequency. The higher the frequency, the more biased towards the velocity from the fused pose and
         // previous pose.
         double velocityAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(icpParameters.getObservationVelocityFusingFrequency(), timeDelta);
         objectLinearVelocity.interpolate(objectLinearVelocity, linearVelocityFromMeasurement, velocityAlpha);
         objectAngularVelocity.interpolate(objectAngularVelocity, angularVelocityFromMeasurement, velocityAlpha);

         objectPose.set(fusedPose);
      }
   }

   public FramePose3DReadOnly getMostRecentFusedPose()
   {
      return objectPose;
   }

   public FramePose3DReadOnly predictObjectPoseNow()
   {
      double currentTime = Conversions.nanosecondsToSeconds(System.nanoTime());
      return predictObjectPoseCurrentVelocities(currentTime - mostRecentMeasurementTime);
   }

   private FramePose3DReadOnly predictObjectPoseCurrentVelocities(double timeDelta)
   {
      FramePose3D posePrediction = new FramePose3D();
      KSTTools.integrateLinearVelocity(timeDelta, objectPose.getPosition(), objectLinearVelocity, posePrediction.getPosition());
      KSTTools.integrateAngularVelocity(timeDelta, objectPose.getOrientation(), objectAngularVelocity, posePrediction.getOrientation());

      return posePrediction;
   }

   private FramePose3DReadOnly fuseMeasuredAndPredictedPose(Pose3DReadOnly measuredObjectPose, FramePose3DReadOnly predictedObjectPose, double timeDelta)
   {
      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(icpParameters.getObservationPoseFusingFrequency(), timeDelta);

      FramePose3D fusedPose = new FramePose3D();
      fusedPose.getRotation().interpolate(measuredObjectPose.getRotation(), predictedObjectPose.getRotation(), alpha);
      fusedPose.getTranslation().interpolate(measuredObjectPose.getTranslation(), predictedObjectPose.getTranslation(), alpha);

      return fusedPose;
   }

   private FrameVector3DReadOnly computeLinearVelocity(FramePose3DReadOnly newPose, FramePose3DReadOnly oldPose, double timeDelta)
   {
      FrameVector3D linearVelocity = new FrameVector3D();
      KSTTools.computeLinearVelocity(timeDelta, oldPose.getPosition(), newPose.getPosition(), linearVelocity);
      return linearVelocity;
   }

   private FrameVector3DReadOnly computeAngularVelocity(FramePose3DReadOnly newPose, FramePose3DReadOnly oldPose, double timeDelta)
   {
      FrameVector3D angularVelocity = new FrameVector3D();
      KSTTools.computeAngularVelocity(timeDelta, oldPose.getOrientation(), newPose.getOrientation(), angularVelocity);
      return angularVelocity;
   }
}
