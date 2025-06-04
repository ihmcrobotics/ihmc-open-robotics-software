package us.ihmc.perception.detections.foundationPose;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class FoundationPoseTools
{
   private static final Map<String, String> YOLO_CLASS_TO_OBJECT_MESH_MAP
         = Map.of("bottle", "Mustard.obj",
                  "door_panel", "DoorPanel.obj",
                  "charge", "Charge.obj",
                  "traffic_barrier", "Barrier.obj");

   public static Map<String, String> getYOLOClassToObjectMeshMap()
   {
      return new HashMap<>(YOLO_CLASS_TO_OBJECT_MESH_MAP);
   }

   public static Set<String> getAvailableMeshes()
   {
      return new HashSet<>(YOLO_CLASS_TO_OBJECT_MESH_MAP.values());
   }

   public static Set<String> getTrackableYOLOClasses()
   {
      return new HashSet<>(YOLO_CLASS_TO_OBJECT_MESH_MAP.keySet());
   }
}
