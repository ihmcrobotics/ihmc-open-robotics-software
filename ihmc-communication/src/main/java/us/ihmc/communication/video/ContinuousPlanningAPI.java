package us.ihmc.communication.video;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.RigidBodyTransformMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;

public class ContinuousPlanningAPI
{
    public static final String IHMC_TOPIC_PREFIX = "ihmc";
    public static final ROS2Topic<?> IHMC_ROOT = new ROS2Topic<>().withPrefix(IHMC_TOPIC_PREFIX);

    public static final SideDependentList<ROS2Topic<RigidBodyTransformMessage>> FOOTSTEP_PLANNING_START = new SideDependentList<>(
          IHMC_ROOT.withModule("continuous_planning").withType(RigidBodyTransformMessage.class).withSuffix("footstep_planning_start_left"),
            IHMC_ROOT.withModule("continuous_planning").withType(RigidBodyTransformMessage.class).withSuffix("footstep_planning_start_right")
    );

    public static final SideDependentList<ROS2Topic<RigidBodyTransformMessage>> FOOTSTEP_PLANNING_GOAL = new SideDependentList<>(
          IHMC_ROOT.withModule("continuous_planning").withType(RigidBodyTransformMessage.class).withSuffix("footstep_planning_goal_left"),
            IHMC_ROOT.withModule("continuous_planning").withType(RigidBodyTransformMessage.class).withSuffix("footstep_planning_goal_right")
    );

    public static final ROS2Topic<FootstepDataListMessage> PLANNED_FOOTSTEPS = IHMC_ROOT.withModule("continuous_planning").withType(FootstepDataListMessage.class).withSuffix("planned_footsteps");
    public static final ROS2Topic<PoseListMessage> START_AND_GOAL_FOOTSTEPS = IHMC_ROOT.withModule("continuous_planning").withType(PoseListMessage.class).withSuffix("start_and_goal");
}
