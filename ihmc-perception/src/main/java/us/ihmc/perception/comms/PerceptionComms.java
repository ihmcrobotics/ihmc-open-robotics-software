package us.ihmc.perception.comms;

import ihmc_common_msgs.msg.dds.StoredPropertySetMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.ros2.ROS2Topic;

public class PerceptionComms
{
   private static final String MODULE_NAME = "perception";
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule(MODULE_NAME);

   public static final StoredPropertySetROS2TopicPair PERSPECTIVE_RAPID_REGION_PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME, "perspective_rapid_region_parameters");
   public static final ROS2Topic<StoredPropertySetMessage> PERSPECTIVE_RAPID_REGIONS_PARAMETERS_COMMAND = PERSPECTIVE_RAPID_REGION_PARAMETERS.getCommandTopic();
   public static final ROS2Topic<StoredPropertySetMessage> PERSPECTIVE_RAPID_REGIONS_PARAMETERS_STATUS = PERSPECTIVE_RAPID_REGION_PARAMETERS.getStatusTopic();

   public static final StoredPropertySetROS2TopicPair PERSPECTIVE_CONVEX_HULL_FACTORY_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME, "perspective_convex_hull_factory_parameters");
   public static final ROS2Topic<StoredPropertySetMessage> PERSPECTIVE_CONVEX_HULL_FACTORY_PARAMETERS_COMMAND = PERSPECTIVE_CONVEX_HULL_FACTORY_PARAMETERS.getCommandTopic();
   public static final ROS2Topic<StoredPropertySetMessage> PERSPECTIVE_CONVEX_HULL_FACTORY_PARAMETERS_STATUS = PERSPECTIVE_CONVEX_HULL_FACTORY_PARAMETERS.getStatusTopic();

   public static final StoredPropertySetROS2TopicPair PERSPECTIVE_POLYGONIZER_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME, "perspective_polygonizer_parameters");
   public static final ROS2Topic<StoredPropertySetMessage> PERSPECTIVE_POLYGONIZER_PARAMETERS_COMMAND = PERSPECTIVE_POLYGONIZER_PARAMETERS.getCommandTopic();
   public static final ROS2Topic<StoredPropertySetMessage> PERSPECTIVE_POLYGONIZER_PARAMETERS_STATUS = PERSPECTIVE_POLYGONIZER_PARAMETERS.getStatusTopic();

   public static final StoredPropertySetROS2TopicPair PERSPECTIVE_PLANAR_REGION_MAPPING_PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME, "perspective_planar_region_mapping_parameters");
   public static final ROS2Topic<StoredPropertySetMessage> PERSPECTIVE_PLANAR_REGION_MAPPING_PARAMETERS_COMMAND = PERSPECTIVE_PLANAR_REGION_MAPPING_PARAMETERS.getCommandTopic();
   public static final ROS2Topic<StoredPropertySetMessage> PERSPECTIVE_PLANAR_REGION_MAPPING_PARAMETERS_STATUS = PERSPECTIVE_PLANAR_REGION_MAPPING_PARAMETERS.getStatusTopic();

   public static final StoredPropertySetROS2TopicPair SPHERICAL_RAPID_REGION_PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME, "spherical_rapid_region_parameters");
   public static final ROS2Topic<StoredPropertySetMessage> SPHERICAL_RAPID_REGIONS_PARAMETERS_COMMAND = SPHERICAL_RAPID_REGION_PARAMETERS.getCommandTopic();
   public static final ROS2Topic<StoredPropertySetMessage> SPHERICAL_RAPID_REGIONS_PARAMETERS_STATUS = SPHERICAL_RAPID_REGION_PARAMETERS.getStatusTopic();

   public static final StoredPropertySetROS2TopicPair SPHERICAL_CONVEX_HULL_FACTORY_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME, "spherical_convex_hull_factory_parameters");
   public static final ROS2Topic<StoredPropertySetMessage> SPHERICAL_CONVEX_HULL_FACTORY_PARAMETERS_COMMAND = SPHERICAL_CONVEX_HULL_FACTORY_PARAMETERS.getCommandTopic();
   public static final ROS2Topic<StoredPropertySetMessage> SPHERICAL_CONVEX_HULL_FACTORY_PARAMETERS_STATUS = SPHERICAL_CONVEX_HULL_FACTORY_PARAMETERS.getStatusTopic();

   public static final StoredPropertySetROS2TopicPair SPHERICAL_POLYGONIZER_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME, "spherical_polygonizer_parameters");
   public static final ROS2Topic<StoredPropertySetMessage> SPHERICAL_POLYGONIZER_PARAMETERS_COMMAND = SPHERICAL_POLYGONIZER_PARAMETERS.getCommandTopic();
   public static final ROS2Topic<StoredPropertySetMessage> SPHERICAL_POLYGONIZER_PARAMETERS_STATUS = SPHERICAL_POLYGONIZER_PARAMETERS.getStatusTopic();


   public static final StoredPropertySetROS2TopicPair SPHERICAL_PLANAR_REGION_MAPPING_PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME, "perspective_planar_region_mapping_parameters");
   public static final ROS2Topic<StoredPropertySetMessage> PARAMETERS_COMMAND = SPHERICAL_PLANAR_REGION_MAPPING_PARAMETERS.getCommandTopic();
   public static final ROS2Topic<StoredPropertySetMessage> PARAMETERS_STATUS = SPHERICAL_PLANAR_REGION_MAPPING_PARAMETERS.getStatusTopic();

   public static final String DEBUG_EXTRACTION_IMAGE = "/gpu_planar_region_extraction/debug_extraction_image";

}
