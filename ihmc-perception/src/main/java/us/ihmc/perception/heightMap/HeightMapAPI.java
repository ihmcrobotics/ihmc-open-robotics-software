package us.ihmc.perception.heightMap;

import ihmc_common_msgs.msg.dds.StoredPropertySetMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.ros2.ROS2Topic;

public class HeightMapAPI
{
   private static final String MODULE_NAME = "cpu_height_map";
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule(MODULE_NAME);
   public static final StoredPropertySetROS2TopicPair PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME, "parameters");
   public static final StoredPropertySetROS2TopicPair FILTER_PARAMETERS = new StoredPropertySetROS2TopicPair(MODULE_NAME, "filter_parameters");
}
