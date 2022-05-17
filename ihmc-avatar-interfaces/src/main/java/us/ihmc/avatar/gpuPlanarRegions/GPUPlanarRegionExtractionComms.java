package us.ihmc.avatar.gpuPlanarRegions;

import controller_msgs.msg.dds.StoredPropertySetMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;

public class GPUPlanarRegionExtractionComms
{
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("gpu_planar_region_extraction");
   private static final ROS2Topic<StoredPropertySetMessage> PARAMETERS = BASE_TOPIC.withType(StoredPropertySetMessage.class).withSuffix("parameters");
   public static final ROS2Topic<StoredPropertySetMessage> PARAMETERS_INPUT = PARAMETERS.withInput();
   public static final ROS2Topic<StoredPropertySetMessage> PARAMETERS_OUTPUT = PARAMETERS.withOutput();
   private static final ROS2Topic<StoredPropertySetMessage> CONVEX_HULL_FACTORY_PARAMETERS
         = BASE_TOPIC.withType(StoredPropertySetMessage.class).withSuffix("convex_hull_factory_parameters");
   public static final ROS2Topic<StoredPropertySetMessage> CONVEX_HULL_FACTORY_PARAMETERS_INPUT = CONVEX_HULL_FACTORY_PARAMETERS.withInput();
   public static final ROS2Topic<StoredPropertySetMessage> CONVEX_HULL_FACTORY_PARAMETERS_OUTPUT = CONVEX_HULL_FACTORY_PARAMETERS.withOutput();
   private static final ROS2Topic<StoredPropertySetMessage> POLYGONIZER_PARAMETERS
         = BASE_TOPIC.withType(StoredPropertySetMessage.class).withSuffix("polygonizer_parameters");
   public static final ROS2Topic<StoredPropertySetMessage> POLYGONIZER_PARAMETERS_INPUT = POLYGONIZER_PARAMETERS.withInput();
   public static final ROS2Topic<StoredPropertySetMessage> POLYGONIZER_PARAMETERS_OUTPUT = POLYGONIZER_PARAMETERS.withOutput();
   public static final ROS2Topic<Empty> RECONNECT_ROS1_NODE = BASE_TOPIC.withType(Empty.class).withSuffix("reconnect_ros1_node");

   public static final String DEBUG_EXTRACTION_IMAGE = "/gpu_planar_region_extraction/debug_extraction_image";

}
