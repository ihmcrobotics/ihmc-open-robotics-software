package us.ihmc.behaviors.door;

import us.ihmc.communication.DeprecatedAPIs;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;

public class DoorBehaviorAPI
{
   private static final String MODULE_NAME = DeprecatedAPIs.BEHAVIOR_MODULE_NAME + "/door_behavior";
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule(MODULE_NAME);
}
