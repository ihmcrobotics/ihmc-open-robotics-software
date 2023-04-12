package us.ihmc.perception.scene;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;

public class SceneObjectAPI
{
   public static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("scene_object");
}
