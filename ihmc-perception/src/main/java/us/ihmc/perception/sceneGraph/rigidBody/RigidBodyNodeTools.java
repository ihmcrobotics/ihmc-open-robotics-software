package us.ihmc.perception.sceneGraph.rigidBody;

import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;

public class RigidBodyNodeTools
{
   public static boolean detectionIsTrashCan(PersistentDetection detection)
   {
      Class<?> detectionClass = detection.getInstantDetectionClass();
      if (detectionClass.equals(YOLOv8InstantDetection.class))
      {
         return detection.getDetectedObjectName().contains("trash_can");
      }
      return false;
   }
}
