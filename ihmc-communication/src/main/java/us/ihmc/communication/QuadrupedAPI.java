package us.ihmc.communication;

import us.ihmc.ros2.ROS2Topic;

public class QuadrupedAPI
{
   public static final String QUADRUPED_CONTROL_MODULE_NAME = "quadruped_control";
   public static final ROS2Topic<?> QUADRUPED_CONTROLLER = ROS2Tools.IHMC_ROOT.withModule(QUADRUPED_CONTROL_MODULE_NAME);
   public static final String QUADRUPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME = "quadruped_support_region_publisher";
   public static final ROS2Topic<?> QUADRUPED_SUPPORT_REGION_PUBLISHER = ROS2Tools.IHMC_ROOT.withModule(QUADRUPED_SUPPORT_REGION_PUBLISHER_MODULE_NAME);

   public static ROS2Topic<?> getQuadrupedControllerOutputTopic(String robotName)
   {
      return QUADRUPED_CONTROLLER.withRobot(robotName).withOutput();
   }

   public static ROS2Topic<?> getQuadrupedControllerInputTopic(String robotName)
   {
      return QUADRUPED_CONTROLLER.withRobot(robotName).withInput();
   }
}
