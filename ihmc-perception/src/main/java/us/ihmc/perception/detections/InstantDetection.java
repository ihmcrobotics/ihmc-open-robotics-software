package us.ihmc.perception.detections;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;

import java.time.Instant;

public abstract class InstantDetection
{
   private final String detectionClass;
   private final double confidence;
   private final Pose3DBasics pose;
   private final Instant detectionTime;

   public InstantDetection(String detectionClass, double confidence, Pose3DBasics pose, Instant detectionTime)
   {
      this.detectionClass = detectionClass;
      this.confidence = confidence;
      this.pose = pose;
      this.detectionTime = detectionTime;
   }

   public String getDetectionClass()
   {
      return detectionClass;
   }

   public double getConfidence()
   {
      return confidence;
   }

   public Pose3DBasics getPose()
   {
      return pose;
   }

   public Instant getDetectionTime()
   {
      return detectionTime;
   }
}
