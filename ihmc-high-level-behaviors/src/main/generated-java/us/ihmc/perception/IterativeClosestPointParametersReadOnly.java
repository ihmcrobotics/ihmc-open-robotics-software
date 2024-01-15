package us.ihmc.perception;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.IterativeClosestPointParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface IterativeClosestPointParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean getIgnoreCorrespondencesOnEdges()
   {
      return get(ignoreCorrespondencesOnEdges);
   }

   /**
    * This is the number of corresponding points the algorithm attempts to use when
    * fitting the point clouds.
    */
   default int getCorrespondencesToUse()
   {
      return get(correspondencesToUse);
   }

   /**
    * This is the minimum number of corresponding points the algorithm needs to run.
    */
   default int getMinimumCorrespondences()
   {
      return get(minimumCorrespondences);
   }

   /**
    * When doing point to point matching, this is the number of points used to
    * describe the object
    */
   default int getPointsToDescribeObjectShape()
   {
      return get(pointsToDescribeObjectShape);
   }

   /**
    * If this is true, we use a velocity to predict the next pose of the object.
    */
   default boolean getComputeObjectPoseWithTrack()
   {
      return get(computeObjectPoseWithTrack);
   }

   default boolean getSegmentPointCloudWithObjectShape()
   {
      return get(segmentPointCloudWithObjectShape);
   }

   /**
    * This is the distance from the object to include in the point cloud. If
    * segmenting with the shape, it's the distance to the shape. If not, it's the
    * distance to the centroid
    */
   default double getImageSegmentationRadius()
   {
      return get(imageSegmentationRadius);
   }

   default boolean getPrioritizeCorrespondencesByDistanceInsteadOfRandom()
   {
      return get(prioritizeCorrespondencesByDistanceInsteadOfRandom);
   }

   default double getIterationTransformDiscountFactor()
   {
      return get(iterationTransformDiscountFactor);
   }

   /**
    * The higher this frequency, the more the result will bias towards the incoming
    * measurement.
    */
   default double getObservationPoseFusingFrequency()
   {
      return get(observationPoseFusingFrequency);
   }

   /**
    * The higher this frequency, the more the resulting velocity will bias towards the
    * new velocity
    */
   default double getObservationVelocityFusingFrequency()
   {
      return get(observationVelocityFusingFrequency);
   }

   /**
    * If the prediction error is above this value vs measured, we reset to the
    * measurement.
    */
   default double getTrackPredictionTranslationErrorToReset()
   {
      return get(trackPredictionTranslationErrorToReset);
   }

   /**
    * If the prediction error is above this value vs measured, we reset to the
    * measurement.
    */
   default double getTrackPredictionOrientationErrorToReset()
   {
      return get(trackPredictionOrientationErrorToReset);
   }
}
