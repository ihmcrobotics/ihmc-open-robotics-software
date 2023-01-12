package us.ihmc.perception.comms;

import ihmc_common_msgs.msg.dds.StoredPropertySetMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.ros2.ROS2Topic;

public class GPUPlanarRegionExtractionComms
{
   private static final String MODULE_NAME = "gpu_planar_region_extraction";
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule(MODULE_NAME);
   public static final StoredPropertySetROS2TopicPair PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME, "parameters");
   public static final ROS2Topic<StoredPropertySetMessage> PARAMETERS_COMMAND = PARAMETERS.getCommandTopic();
   public static final ROS2Topic<StoredPropertySetMessage> PARAMETERS_STATUS = PARAMETERS.getStatusTopic();
   public static final StoredPropertySetROS2TopicPair CONVEX_HULL_FACTORY_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME, "convex_hull_factory_parameters");
   public static final ROS2Topic<StoredPropertySetMessage> CONVEX_HULL_FACTORY_PARAMETERS_COMMAND = CONVEX_HULL_FACTORY_PARAMETERS.getCommandTopic();
   public static final ROS2Topic<StoredPropertySetMessage> CONVEX_HULL_FACTORY_PARAMETERS_STATUS = CONVEX_HULL_FACTORY_PARAMETERS.getStatusTopic();
   public static final StoredPropertySetROS2TopicPair POLYGONIZER_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME, "polygonizer_parameters");
   public static final ROS2Topic<StoredPropertySetMessage> POLYGONIZER_PARAMETERS_COMMAND = POLYGONIZER_PARAMETERS.getCommandTopic();
   public static final ROS2Topic<StoredPropertySetMessage> POLYGONIZER_PARAMETERS_STATUS = POLYGONIZER_PARAMETERS.getStatusTopic();
   public static final ROS2Topic<Empty> RECONNECT_ROS1_NODE = BASE_TOPIC.withType(Empty.class).withSuffix("reconnect_ros1_node");

   public static final String DEBUG_EXTRACTION_IMAGE = "/gpu_planar_region_extraction/debug_extraction_image";

}
