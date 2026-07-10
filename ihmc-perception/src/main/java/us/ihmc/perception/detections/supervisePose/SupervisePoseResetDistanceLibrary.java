package us.ihmc.perception.detections.supervisePose;

import java.util.Map;

public class SupervisePoseResetDistanceLibrary
{
   /**
    * Offline-precomputed reset distances in meters.
    * Key format: "<category>/<instance>"
    */
   private static final Map<String, Double> RESET_DISTANCE_MAP = Map.ofEntries(
         Map.entry("bottle/bottle_1", 0.1267),
         Map.entry("bottle/bottle_2", 0.1356),
         Map.entry("bottle/bottle_3", 0.1036),

         Map.entry("charge/charge_1", 0.1753),
         Map.entry("charge/charge_2", 0.1758),

         Map.entry("door_knob/door_knob_1", 0.0475),

         Map.entry("door_lever/door_lever_1", 0.0760),
         Map.entry("door_lever/door_lever_2", 0.0748),

         Map.entry("door_panel/door_panel_1", 1.1546),
         Map.entry("door_panel/door_panel_2", 1.1147),

         Map.entry("door_pull_handle/door_pull_handle_1", 0.2121),
         Map.entry("door_push_bar/door_push_bar_1", 0.3510),

         Map.entry("storage_container/storage_container_1",  0.2742),
         Map.entry("storage_container/storage_container_2", 0.2998),

         Map.entry("traffic_barrier/traffic_barrier_1", 0.6901),

         Map.entry("trash_can/trash_can_1", 0.5536),
         Map.entry("trash_can/trash_can_2", 0.4445),
         Map.entry("trash_can/trash_can_3", 0.3280)
   );

   private static final double DEFAULT_RESET_DISTANCE = 0.10;

   public static double getResetDistance(String category, String instance)
   {
      return RESET_DISTANCE_MAP.getOrDefault(category + "/" + instance, DEFAULT_RESET_DISTANCE);
   }

   public static double getResetDistance(SupervisePoseObject object)
   {
      return getResetDistance(object.category, object.instance);
   }

   private SupervisePoseResetDistanceLibrary()
   {
   }
}