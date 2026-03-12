package us.ihmc.perception.detections.foundationPose;

import java.util.Map;

public class CategoryLevelFoundationPoseResetDistanceLibrary
{
   /**
    * Offline-precomputed reset distances in meters.
    * Key format: "<category>/<instance>"
    */
   private static final Map<String, Double> RESET_DISTANCE_MAP = Map.ofEntries(
         Map.entry("bottle/bottle_1", 0.08),
         Map.entry("bottle/bottle_2", 0.09),
         Map.entry("bottle/bottle_3", 0.10),

         Map.entry("charge/charge_1", 0.07),
         Map.entry("charge/charge_2", 0.07),

         Map.entry("door_knob/door_knob_1", 0.04),

         Map.entry("door_lever/door_lever_1", 0.05),
         Map.entry("door_lever/door_lever_2", 0.05),

         Map.entry("door_panel/door_panel_1", 0.15),
         Map.entry("door_panel/door_panel_2", 0.15),

         Map.entry("door_pull_handle/door_pull_handle_1", 0.05),
         Map.entry("door_push_bar/door_push_bar_1", 0.10),

         Map.entry("storage_container/storage_container_1", 0.12),
         Map.entry("storage_container/storage_container_2", 0.12),

         Map.entry("traffic_barrier/traffic_barrier_1", 0.20),

         Map.entry("trash_can/trash_can_1", 0.18),
         Map.entry("trash_can/trash_can_2", 0.18),
         Map.entry("trash_can/trash_can_3", 0.18)
   );

   private static final double DEFAULT_RESET_DISTANCE = 0.10;

   public static double getResetDistance(String category, String instance)
   {
      return RESET_DISTANCE_MAP.getOrDefault(category + "/" + instance, DEFAULT_RESET_DISTANCE);
   }

   public static double getResetDistance(CategoryLevelFoundationPoseObject object)
   {
      return getResetDistance(object.category, object.instance);
   }

   private CategoryLevelFoundationPoseResetDistanceLibrary()
   {
   }
}