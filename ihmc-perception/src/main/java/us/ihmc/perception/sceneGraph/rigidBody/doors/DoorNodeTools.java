package us.ihmc.perception.sceneGraph.rigidBody.doors;

import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.YOLOv8.YOLOv8InstantDetection;

import java.util.ArrayList;
import java.util.List;

public final class DoorNodeTools
{
   private static final List<String> doorComponentDetectionNames = new ArrayList<>();

   static
   {
      doorComponentDetectionNames.add("YOLODoorLever");
      doorComponentDetectionNames.add("YOLODoorKnob");
      doorComponentDetectionNames.add("YOLOPushBar");
      doorComponentDetectionNames.add("YOLOPullHandle");
      doorComponentDetectionNames.add("YOLODoorPanel");
   }

   public static boolean detectionIsDoorComponent(PersistentDetection detection)
   {
      Class<?> detectionClass = detection.getInstantDetectionClass();
      if (detectionClass.equals(YOLOv8InstantDetection.class))
      {
         return doorComponentDetectionNames.contains(detection.getDetectedObjectClass());
      }
      return false;
   }
}
