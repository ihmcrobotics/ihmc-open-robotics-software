package us.ihmc.avatar.kinematicsSimulation;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.ros2.ROS2NodeInterface;

public class KinematicRobotiqHandSimulator
{
   double trajectoryTime = 0.5;

   public KinematicRobotiqHandSimulator(ROS2NodeInterface ros2Node, String robotName, FullRobotModel fullRobotModel)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, ROS2Tools.getHandConfigurationTopic(robotName), handMessage ->
      {

      });
   }
}
