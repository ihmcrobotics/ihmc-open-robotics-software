package us.ihmc.gdx.ui.behaviors.registry;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2NodeInterface;

public interface GDXBehaviorUIInterfaceConstructor
{
   GDXBehaviorUIInterface create(ROS2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel);
}
