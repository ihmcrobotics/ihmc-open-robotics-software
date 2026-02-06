package us.ihmc.perception.detections;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;

import java.time.Instant;

/**
 * Represents a single detection directly and immediately from a perception algorithm which does
 * not already have stability filtering, history, or persistent tracking built in.
 * A new instance of this is created for each detected element for each frame.
 *<p>
 * The main subclasses are:
 * <ul>
 *    <li>{@link us.ihmc.perception.detections.yolo.YOLOv8InstantDetection}</li>
 *    <li>{@link us.ihmc.perception.detections.foundationPose.FoundationPoseInstantDetection}</li>
 * </ul>
 */
public class InstantDetection
{
   private static final double EPSILON = 1E-7;

   /** The object's identifying class (e.g. the object class identified by YOLO or simply the ArUco marker number*/
   private final String detectedObjectClass;
   /** Colloquial name of the detected object (e.g. "Shoe", "Door Lever", etc.)*/
   private final String detectedObjectName;
   private final double confidence;
   /** The pose of the object at the time of detection **/
   private final Pose3DReadOnly pose;
   private final Instant detectionTime;

   public InstantDetection(String detectedObjectClass, double confidence, Pose3DReadOnly pose, Instant detectionTime)
   {
      this(detectedObjectClass, detectedObjectClass, confidence, pose, detectionTime);
   }

   public InstantDetection(String detectedObjectClass, String detectedObjectName, double confidence, Pose3DReadOnly pose, Instant detectionTime)
   {
      this.detectedObjectClass = detectedObjectClass;
      this.detectedObjectName = detectedObjectName;
      this.confidence = confidence;
      this.pose = new Pose3D(pose);
      this.detectionTime = detectionTime;
   }

   public String getDetectedObjectClass()
   {
      return detectedObjectClass;
   }

   public String getDetectedObjectName()
   {
      return detectedObjectName;
   }

   public double getConfidence()
   {
      return confidence;
   }

   public Pose3DReadOnly getPose()
   {
      return pose;
   }

   public Instant getDetectionTime()
   {
      return detectionTime;
   }

   @Override
   public boolean equals(Object other)
   {
      if (this == other)
         return true;

      if (other instanceof InstantDetection otherDetection)
      {
         return detectionTime.equals(otherDetection.detectionTime)
                && detectedObjectClass.equals(otherDetection.detectedObjectClass)
                && detectedObjectName.equals(otherDetection.detectedObjectName)
                && MathTools.epsilonEquals(confidence, otherDetection.confidence, EPSILON)
                && pose.epsilonEquals(otherDetection.pose, EPSILON);
      }
      else
         return false;
   }

   public void destroy()
   {

   }
}
