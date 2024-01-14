package us.ihmc.perception;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;

public class IterativeClosestPointObjectTrack
{
   private static double observationFusingFrequency = 50.0;
   private static double velocityFusingFrequency = 25.0;

   private final FramePose3D objectPose = new FramePose3D();
   private final FrameVector3D objectLinearVelocity = new FrameVector3D();
   private final FrameVector3D objectAngularVelocity = new FrameVector3D();

   private double mostRecentMeasurementTime;

   public IterativeClosestPointObjectTrack()
   {}

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

      FramePose3DReadOnly posePrediction = predictObjectPoseCurrentVelocities(timeDelta);

      FramePose3DReadOnly fusedPose = fuseMeasuredAndPredictedPose(measuredObjectPose, posePrediction, timeDelta);

      FrameVector3DReadOnly linearVelocityFromMeasurement = computeLinearVelocity(fusedPose, objectPose, timeDelta);
      FrameVector3DReadOnly angularVelocityFromMeasurement = computeAngularVelocity(fusedPose, objectPose, timeDelta);

      // update the velocity measurements using the fusing frequency.
      double velocityALpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(velocityFusingFrequency, timeDelta);
      objectLinearVelocity.interpolate(objectLinearVelocity, linearVelocityFromMeasurement, velocityALpha);
      objectAngularVelocity.interpolate(objectAngularVelocity, angularVelocityFromMeasurement, velocityALpha);

      this.objectPose.set(fusedPose);
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
      FramePose3D posePrediction = new FramePose3D(objectPose);
      KSTTools.integrateLinearVelocity(timeDelta, objectPose.getPosition(), objectLinearVelocity, posePrediction.getPosition());
      KSTTools.integrateAngularVelocity(timeDelta, objectPose.getOrientation(), objectAngularVelocity, posePrediction.getOrientation());

      return posePrediction;
   }

   private FramePose3DReadOnly fuseMeasuredAndPredictedPose(Pose3DReadOnly measuredObjectPose, FramePose3DReadOnly predictedObjectPose, double timeDelta)
   {
      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(observationFusingFrequency, timeDelta);

      FramePose3D fusedPose = new FramePose3D();
      fusedPose.getRotation().interpolate(predictedObjectPose.getRotation(), measuredObjectPose.getRotation(), alpha);
      fusedPose.getTranslation().interpolate(predictedObjectPose.getTranslation(), measuredObjectPose.getTranslation(), alpha);

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
