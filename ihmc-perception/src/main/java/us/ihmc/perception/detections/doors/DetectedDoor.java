package us.ihmc.perception.detections.doors;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.perception.detections.InstantDetection;

import java.time.Instant;

import static us.ihmc.perception.detections.doors.DoorDetectionManager.DOOR_STRING;
import static us.ihmc.perception.detections.doors.DoorDetectionManager.PANEL_STRING;

public class DetectedDoor
{
   private String openingHardwareName;
   private final Pose3D openingHardwarePose;

   private final Pose3D panelPose;

   private Instant lastDetectionTime;

   public DetectedDoor()
   {
      openingHardwarePose = new Pose3D();
      openingHardwarePose.setToNaN();

      panelPose = new Pose3D();
      panelPose.setToNaN();
   }

   /* package-private */ double distanceSquared(InstantDetection detection)
   {
      String detectedClass = detection.getDetectedObjectClass().toLowerCase();

      if (detectedClass.contains(PANEL_STRING))
         return detection.getPose().getPosition().distanceSquared(getPanelPose().getPosition());
      else if (detectedClass.startsWith(DOOR_STRING))
         return detection.getPose().getPosition().distanceSquared(getOpeningHardwarePose().getPosition());

      return Double.POSITIVE_INFINITY;
   }

   /* package-private */ void updateDetection(InstantDetection newDetection)
   {
      String detectedClass = newDetection.getDetectedObjectClass().toLowerCase();

      if (detectedClass.contains(PANEL_STRING))
      {
         panelPose.getPosition().set(newDetection.getPose().getPosition());
      }
      else if (detectedClass.startsWith(DOOR_STRING))
      {
         openingHardwareName = newDetection.getDetectedObjectClass();
         openingHardwarePose.getPosition().set(newDetection.getPose().getPosition());
      }
      else
      {
         return;
      }

      if (newDetection.getDetectionTime().isAfter(lastDetectionTime))
         lastDetectionTime = newDetection.getDetectionTime();
   }

   public String getOpeningHardwareName()
   {
      return openingHardwareName;
   }

   public Pose3DReadOnly getOpeningHardwarePose()
   {
      return openingHardwarePose;
   }

   public Pose3DReadOnly getPanelPose()
   {
      return panelPose;
   }

   public Instant getLastDetectedTime()
   {
      return lastDetectionTime;
   }
}
