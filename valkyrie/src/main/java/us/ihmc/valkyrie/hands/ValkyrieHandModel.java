package us.ihmc.valkyrie.hands;

import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.yoVariables.providers.DoubleProvider;

public interface ValkyrieHandModel extends HandModel
{
   ValkyrieHandVersion getHandVersion();

   ValkyrieHandController newSimulatedHandController(RobotSide robotSide,
                                                     FullHumanoidRobotModel fullRobotModel,
                                                     JointDesiredOutputListBasics jointDesiredOutputList,
                                                     DoubleProvider yoTime,
                                                     RealtimeROS2Node realtimeROS2Node,
                                                     ROS2Topic<?> inputTopic);
}
