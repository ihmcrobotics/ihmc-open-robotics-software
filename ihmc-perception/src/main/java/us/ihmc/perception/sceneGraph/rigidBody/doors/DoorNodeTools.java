package us.ihmc.perception.sceneGraph.rigidBody.doors;

import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.YOLOv8.YOLOv8InstantDetection;

import java.util.ArrayList;
import java.util.List;

public final class DoorNodeTools
{
   private static final List<String> doorOpeningMechanismDetectionNames = new ArrayList<>();
   private static final String doorPanelDetectionName = "YOLODoorPanel";
   private static final List<String> doorComponentDetectionNames = new ArrayList<>();

   static
   {
      doorOpeningMechanismDetectionNames.add("YOLODoorLever");
      doorOpeningMechanismDetectionNames.add("YOLODoorKnob");
      doorOpeningMechanismDetectionNames.add("YOLOPushBar");
      doorOpeningMechanismDetectionNames.add("YOLOPullHandle");

      doorComponentDetectionNames.addAll(doorOpeningMechanismDetectionNames);
      doorComponentDetectionNames.add(doorPanelDetectionName);
   }

   public static boolean detectionIsDoorComponent(PersistentDetection detection)
   {
      Class<?> detectionClass = detection.getInstantDetectionClass();
      if (detectionClass.equals(YOLOv8InstantDetection.class))
      {
         return doorComponentDetectionNames.contains(detection.getDetectedObjectName());
      }
      return false;
   }

   public static boolean detectionIsDoorOpeningMechanism(PersistentDetection detection)
   {
      Class<?> detectionClass = detection.getInstantDetectionClass();
      if (detectionClass.equals(YOLOv8InstantDetection.class))
      {
         return doorOpeningMechanismDetectionNames.contains(detection.getDetectedObjectName());
      }
      return false;
   }

   public static boolean detectionIsDoorPanel(PersistentDetection detection)
   {
      Class<?> detectionClass = detection.getInstantDetectionClass();
      if (detectionClass.equals(YOLOv8InstantDetection.class))
      {
         return doorPanelDetectionName.contains(detection.getDetectedObjectName());
      }
      return false;
   }
}
