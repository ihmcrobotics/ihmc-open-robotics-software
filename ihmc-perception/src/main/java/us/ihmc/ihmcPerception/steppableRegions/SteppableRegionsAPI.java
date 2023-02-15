package us.ihmc.ihmcPerception.steppableRegions;

import perception_msgs.msg.dds.SteppableRegionDebugImagesMessage;
import perception_msgs.msg.dds.SteppableRegionsListCollectionMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.ros2.ROS2Topic;

public class SteppableRegionsAPI
{
   public static final String STEPPABLE_REGIONS_MODULE_NAME = "steppable_regions";

   public static final ROS2Topic<?> STEPPABLE_REGIONS_MODULE = ROS2Tools.IHMC_ROOT.withModule(STEPPABLE_REGIONS_MODULE_NAME);

   public static final ROS2Topic<SteppableRegionsListCollectionMessage> STEPPABLE_REGIONS_OUTPUT = STEPPABLE_REGIONS_MODULE.withOutput().withTypeName(SteppableRegionsListCollectionMessage.class);
   public static final ROS2Topic<SteppableRegionDebugImagesMessage> STEPPABLE_REGIONS_DEBUG_OUTPUT = STEPPABLE_REGIONS_MODULE.withOutput().withTypeName(SteppableRegionDebugImagesMessage.class);
   public static final StoredPropertySetROS2TopicPair PARAMETERS = new StoredPropertySetROS2TopicPair(STEPPABLE_REGIONS_MODULE_NAME, "parameters");

}
