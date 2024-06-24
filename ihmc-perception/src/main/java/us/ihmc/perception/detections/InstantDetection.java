package us.ihmc.perception.detections;

import perception_msgs.msg.dds.InstantDetectionMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;

import java.time.Instant;
import java.util.UUID;

public abstract class InstantDetection
{
   public static final UUID UNMATCHED_DETECTION_ID = new UUID(0L, 0L);
   private static final double EPSILON = 1E-7;

   private final String detectedObjectClass;
   private final String detectedObjectName;
   private final double confidence;
   private final Pose3D pose;
   private final Instant detectionTime;
   private UUID persistentDetectionID = UNMATCHED_DETECTION_ID;

   public InstantDetection(String detectedObjectClass, double confidence, Pose3D pose, Instant detectionTime)
   {
      this(detectedObjectClass, detectedObjectClass, confidence, pose, detectionTime);
   }

   public InstantDetection(String detectedObjectClass, String detectedObjectName, double confidence, Pose3D pose, Instant detectionTime)
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

   public Pose3D getPose()
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
}
