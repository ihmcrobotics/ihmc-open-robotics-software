package us.ihmc.communication.video;

import controller_msgs.msg.dds.FootstepDataListMessage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.ros2.ROS2Topic;

public class ContinuousPlanningAPI
{
    public static final String IHMC_TOPIC_PREFIX = "ihmc";
    public static final ROS2Topic<?> IHMC_ROOT = new ROS2Topic<>().withPrefix(IHMC_TOPIC_PREFIX);

    public static final ROS2Topic<FootstepDataListMessage> PLANNED_FOOTSTEPS = IHMC_ROOT.withModule("continuous_planning").withType(FootstepDataListMessage.class).withSuffix("planned_footsteps");
}
