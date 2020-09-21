package us.ihmc.humanoidBehaviors.ui;

import javafx.scene.Group;
import javafx.scene.SubScene;
import javafx.scene.layout.Pane;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2NodeInterface;

public abstract class BehaviorUIInterface extends Group
{
   public abstract void init(SubScene sceneNode, Pane visualizationPane, ROS2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel);

   public abstract void setEnabled(boolean enabled);

   public abstract void destroy();
}
