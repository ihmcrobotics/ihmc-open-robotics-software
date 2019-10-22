package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.Ros2Node;

public class BehaviorHelperBuilder
{
   private final DRCRobotModel robotModel;
   private final Messager messager;
   private final Ros2Node ros2Node;

   public BehaviorHelperBuilder(DRCRobotModel robotModel, Messager messager, Ros2Node ros2Node)
   {
      this.robotModel = robotModel;
      this.messager = messager;
      this.ros2Node = ros2Node;
   }

   public BehaviorHelper build()
   {
      return new BehaviorHelper(messager, robotModel, ros2Node);
   }
}
