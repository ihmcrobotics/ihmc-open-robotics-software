package us.ihmc.perception.comms;

import ihmc_common_msgs.msg.dds.StoredPropertySetMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.ros2.ROS2Topic;

public class PerceptionComms
{
   private static final String MODULE_NAME = "perception";

   public static final StoredPropertySetROS2TopicPair PERSPECTIVE_RAPID_REGION_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME,"perspective_rapid_region_parameters");

   public static final StoredPropertySetROS2TopicPair PERSPECTIVE_CONVEX_HULL_FACTORY_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME,"perspective_convex_hull_factory_parameters");

   public static final StoredPropertySetROS2TopicPair PERSPECTIVE_POLYGONIZER_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME,"perspective_polygonizer_parameters");

   public static final StoredPropertySetROS2TopicPair PERSPECTIVE_PLANAR_REGION_MAPPING_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME,"perspective_planar_region_mapping_parameters");

   public static final StoredPropertySetROS2TopicPair SPHERICAL_RAPID_REGION_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME,"spherical_rapid_region_parameters");

   public static final StoredPropertySetROS2TopicPair SPHERICAL_CONVEX_HULL_FACTORY_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME,"spherical_convex_hull_factory_parameters");

   public static final StoredPropertySetROS2TopicPair SPHERICAL_POLYGONIZER_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME,"spherical_polygonizer_parameters");

   public static final StoredPropertySetROS2TopicPair SPHERICAL_PLANAR_REGION_MAPPING_PARAMETERS
         = new StoredPropertySetROS2TopicPair(MODULE_NAME,"perspective_planar_region_mapping_parameters");
}
