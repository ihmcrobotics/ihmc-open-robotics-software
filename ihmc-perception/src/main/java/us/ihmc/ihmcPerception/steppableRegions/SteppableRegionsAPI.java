package us.ihmc.ihmcPerception.steppableRegions;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;

public class SteppableRegionsAPI
{
   public static final StoredPropertySetROS2TopicPair PARAMETERS = new StoredPropertySetROS2TopicPair(ROS2Tools.STEPPABLE_REGIONS_MODULE_NAME, "parameters");

}
