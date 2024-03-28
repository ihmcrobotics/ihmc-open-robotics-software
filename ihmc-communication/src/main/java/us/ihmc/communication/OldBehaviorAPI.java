package us.ihmc.communication;

import us.ihmc.ros2.ROS2Topic;

@Deprecated
public class OldBehaviorAPI
{
   public static final ROS2Topic<?> BEHAVIOR_MODULE = ROS2Tools.IHMC_ROOT.withModule(ROS2Tools.BEHAVIOR_MODULE_NAME);
}
