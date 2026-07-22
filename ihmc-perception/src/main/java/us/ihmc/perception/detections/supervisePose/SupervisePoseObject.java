package us.ihmc.perception.detections.supervisePose;

import java.nio.file.Path;
import java.nio.file.Files;

public enum SupervisePoseObject
{
   TRAFFIC_BARRIER_1("Traffic Barrier 1", "traffic_barrier", "traffic_barrier_1", "traffic_barrier"),
   BOTTLE_1("Bottle 1", "bottle", "bottle_1", "bottle"),
   BOTTLE_2("Bottle 2", "bottle", "bottle_2", "bottle"),
   BOTTLE_3("Bottle 3", "bottle", "bottle_3", "bottle"),
   CHARGE_1("Charge 1", "charge", "charge_1", "charge"),
   CHARGE_2("Charge 2", "charge", "charge_2", "charge"),
   DOOR_PANEL_1("Door Panel 1", "door_panel", "door_panel_1", "door_panel"),
   DOOR_PANEL_2("Door Panel 2", "door_panel", "door_panel_2", "door_panel"),
   DOOR_PULL_HANDLE_1("Door Pull Handle 1", "door_pull_handle", "door_pull_handle_1", "door_pull_handle"),
   DOOR_KNOB_1("Door Knob 1", "door_knob", "door_knob_1", "door_knob"),
   DOOR_LEVER_1("Door Lever 1", "door_lever", "door_lever_1", "door_lever"),
   DOOR_LEVER_2("Door Lever 2", "door_lever", "door_lever_2", "door_lever"),
   DOOR_PUSH_BAR_1("Door Push Bar 1", "door_push_bar", "door_push_bar_1", "door_push_bar"),
   STORAGE_CONTAINER_1("Storage Container 1", "storage_container", "storage_container_1", "storage_container"),
   STORAGE_CONTAINER_2("Storage Container 2", "storage_container", "storage_container_2", "storage_container"),
   TRASH_CAN_1("Trash Can 1", "trash_can", "trash_can_1", "trash_can"),
   TRASH_CAN_2("Trash Can 2", "trash_can", "trash_can_2", "trash_can"),
   TRASH_CAN_3("Trash Can 3", "trash_can", "trash_can_3", "trash_can");

   public static final SupervisePoseObject[] VALUES = values();

   public final String titleCaseName;
   public final String category;
   public final String instance;
   public final String yoloClass;
   public final SupervisePoseAPI.SupervisePoseTopics topics;
   private final Path meshPath;

   SupervisePoseObject(String titleCaseName, String category, String instance, String yoloClass)
   {
      this.titleCaseName = titleCaseName;
      this.category = category;
      this.instance = instance;
      this.yoloClass = yoloClass;
      topics = SupervisePoseAPI.topics(category, instance);
      meshPath = createMeshPath(category, instance);
   }

   /**
    * Builds the mesh path using the directory organization:
    *
    * meshes/
    * └── category/
    *     └── instance/
    *         └── instance.obj
    *
    * Example:
    *
    * sparse_meshes/bottle/bottle_1/bottle_1.obj
    */
   private static Path createMeshPath(String category, String instance)
   {
      Path meshRoot = Path.of(System.getProperty("user.home"), "alexander", "repository-group", "ihmc-isaac-ros", "isaac_ros_assets", "sparse_meshes");

      Path resolvedMeshPath = meshRoot.resolve(category).resolve(instance).resolve(instance + ".obj");

      if (!Files.isRegularFile(resolvedMeshPath))
      {
         throw new IllegalArgumentException("SupervisePose mesh file does not exist: " + resolvedMeshPath.toAbsolutePath());
      }

      return resolvedMeshPath;
   }

   public Path getMeshPath()
   {
      return meshPath;
   }

   public static SupervisePoseObject fromCategoryAndInstance(String category, String instance)
   {
      for (SupervisePoseObject object : VALUES)
      {
         if (object.category.equals(category)
             && object.instance.equals(instance))
         {
            return object;
         }
      }

      throw new IllegalArgumentException(
            "No SupervisePoseObject for " + category + "/" + instance);
   }

   public String key()
   {
      return category + "/" + instance;
   }
}