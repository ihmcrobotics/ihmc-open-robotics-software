package us.ihmc.gdx.ui.behaviors.registry;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2NodeInterface;

public abstract class GDXBehaviorUIInterface
{
   private final ROS2NodeInterface ros2Node;
   private final Messager behaviorMessager;
   private final DRCRobotModel robotModel;

   protected GDXBehaviorUIInterface(ROS2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.ros2Node = ros2Node;
      this.behaviorMessager = behaviorMessager;
      this.robotModel = robotModel;
   }

   public abstract void setEnabled(boolean enabled);

   public abstract void destroy();

   protected void enable3DVisualizations()
   {
   }

   protected ROS2NodeInterface getRos2Node()
   {
      return ros2Node;
   }

   protected Messager getBehaviorMessager()
   {
      return behaviorMessager;
   }

   protected DRCRobotModel getRobotModel()
   {
      return robotModel;
   }
}
