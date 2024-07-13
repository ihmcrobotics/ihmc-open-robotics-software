package us.ihmc.perception.detections;

import perception_msgs.msg.dds.InstantDetectionMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;

import java.time.Instant;
import java.util.UUID;

/**
 * Represents a single detection directly and immediately from a perception algorithm which does
 * not already have stability filtering, history, or persistent tracking built in.
 * A new instance of this is created for each detected element for each frame.
 *
 * The main subclasses are:
 * <ul>
 *    <li>{@link us.ihmc.perception.detections.YOLOv8.YOLOv8InstantDetection}</li>
 *    <li>{@link us.ihmc.perception.detections.centerPose.CenterPoseInstantDetection}</li>
 * </ul>
 */
public class InstantDetection
{
   private static final double EPSILON = 1E-7;

   /** The object's identifying class (e.g. {@link us.ihmc.perception.detections.YOLOv8.YOLOv8DetectionClass} or simply the ArUco marker number*/
   private final String detectedObjectClass;
   /** Colloquial name of the detected object (e.g. "Shoe", "Door Lever", etc)*/
   private final String detectedObjectName;
   private final double confidence;
   private final Pose3DReadOnly pose;
   private final Instant detectionTime;
   private UUID persistentDetectionID = PersistentDetection.NULL_DETECTION_ID;

   public InstantDetection(String detectedObjectClass, double confidence, Pose3DReadOnly pose, Instant detectionTime)
   {
      this(detectedObjectClass, detectedObjectClass, confidence, pose, detectionTime);
   }

   public InstantDetection(String detectedObjectClass, String detectedObjectName, double confidence, Pose3DReadOnly pose, Instant detectionTime)
   {
      this.detectedObjectClass = detectedObjectClass;
      this.detectedObjectName = detectedObjectName;
      this.confidence = confidence;
      this.pose = pose;
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

   public UUID getPersistentDetectionID()
   {
      return persistentDetectionID;
   }

   public void setPersistentDetectionID(UUID persistentDetectionID)
   {
      this.persistentDetectionID = persistentDetectionID;
   }

   public void toMessage(InstantDetectionMessage message)
   {
      message.setDetectedObjectClass(detectedObjectClass);
      message.setDetectedObjectName(detectedObjectName);
      message.setConfidence(confidence);
      message.getObjectPose().set(pose);
      MessageTools.toMessage(detectionTime, message.getDetectionTime());
      MessageTools.toMessage(persistentDetectionID, message.getPersistentDetectionId());
   }

   @Override
   public boolean equals(Object other)
   {
      if (this == other)
         return true;

      if (other instanceof InstantDetection otherDetection)
      {
         return detectedObjectClass.equals(otherDetection.detectedObjectClass)
                && detectedObjectName.equals(otherDetection.detectedObjectName)
                && MathTools.epsilonEquals(confidence, otherDetection.confidence, EPSILON)
                && pose.epsilonEquals(otherDetection.pose, EPSILON)
                && detectionTime.equals(otherDetection.detectionTime)
                && persistentDetectionID.equals(otherDetection.persistentDetectionID);
      }
      else
         return false;
   }

   public void destroy()
   {

   }
}
