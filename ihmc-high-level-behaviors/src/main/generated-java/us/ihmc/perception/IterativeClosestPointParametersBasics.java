package us.ihmc.perception;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface IterativeClosestPointParametersBasics extends IterativeClosestPointParametersReadOnly, StoredPropertySetBasics
{
   default void setIgnoreCorrespondencesOnEdges(boolean ignoreCorrespondencesOnEdges)
   {
      set(IterativeClosestPointParameters.ignoreCorrespondencesOnEdges, ignoreCorrespondencesOnEdges);
   }

   /**
    * This is the number of corresponding points the algorithm attempts to use when
    * fitting the point clouds.
    */
   default void setCorrespondencesToUse(int correspondencesToUse)
   {
      set(IterativeClosestPointParameters.correspondencesToUse, correspondencesToUse);
   }

   /**
    * This is the minimum number of corresponding points the algorithm needs to run.
    */
   default void setMinimumCorrespondences(int minimumCorrespondences)
   {
      set(IterativeClosestPointParameters.minimumCorrespondences, minimumCorrespondences);
   }

   /**
    * When doing point to point matching, this is the number of points used to
    * describe the object
    */
   default void setPointsToDescribeObjectShape(int pointsToDescribeObjectShape)
   {
      set(IterativeClosestPointParameters.pointsToDescribeObjectShape, pointsToDescribeObjectShape);
   }

   /**
    * If this is true, we use a velocity to predict the next pose of the object.
    */
   default void setComputeObjectPoseWithTrack(boolean computeObjectPoseWithTrack)
   {
      set(IterativeClosestPointParameters.computeObjectPoseWithTrack, computeObjectPoseWithTrack);
   }

   default void setSegmentPointCloudWithObjectShape(boolean segmentPointCloudWithObjectShape)
   {
      set(IterativeClosestPointParameters.segmentPointCloudWithObjectShape, segmentPointCloudWithObjectShape);
   }

   /**
    * This is the distance from the object to include in the point cloud. If
    * segmenting with the shape, it's the distance to the shape. If not, it's the
    * distance to the centroid
    */
   default void setImageSegmentationRadius(double imageSegmentationRadius)
   {
      set(IterativeClosestPointParameters.imageSegmentationRadius, imageSegmentationRadius);
   }

   default void setPrioritizeCorrespondencesByDistanceInsteadOfRandom(boolean prioritizeCorrespondencesByDistanceInsteadOfRandom)
   {
      set(IterativeClosestPointParameters.prioritizeCorrespondencesByDistanceInsteadOfRandom, prioritizeCorrespondencesByDistanceInsteadOfRandom);
   }

   default void setIterationTransformDiscountFactor(double iterationTransformDiscountFactor)
   {
      set(IterativeClosestPointParameters.iterationTransformDiscountFactor, iterationTransformDiscountFactor);
   }

   /**
    * The higher this frequency, the more the result will bias towards the incoming
    * measurement.
    */
   default void setObservationPoseFusingFrequency(double observationPoseFusingFrequency)
   {
      set(IterativeClosestPointParameters.observationPoseFusingFrequency, observationPoseFusingFrequency);
   }

   /**
    * The higher this frequency, the more the resulting velocity will bias towards the
    * new velocity
    */
   default void setObservationVelocityFusingFrequency(double observationVelocityFusingFrequency)
   {
      set(IterativeClosestPointParameters.observationVelocityFusingFrequency, observationVelocityFusingFrequency);
   }

   /**
    * If the prediction error is above this value vs measured, we reset to the
    * measurement.
    */
   default void setTrackPredictionTranslationErrorToReset(double trackPredictionTranslationErrorToReset)
   {
      set(IterativeClosestPointParameters.trackPredictionTranslationErrorToReset, trackPredictionTranslationErrorToReset);
   }

   /**
    * If the prediction error is above this value vs measured, we reset to the
    * measurement.
    */
   default void setTrackPredictionOrientationErrorToReset(double trackPredictionOrientationErrorToReset)
   {
      set(IterativeClosestPointParameters.trackPredictionOrientationErrorToReset, trackPredictionOrientationErrorToReset);
   }
}
