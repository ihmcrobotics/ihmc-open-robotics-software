package us.ihmc.humanoidBehaviors;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.Ros2Node;

public class BehaviorBuilder
{
   private final BehaviorStatics statics;

   private BehaviorInterface behavior;

   public BehaviorBuilder(BehaviorStatics statics)
   {
      this.statics = statics;
   }

   public void build(DRCRobotModel robotModel, Messager messager, Ros2Node ros2Node)
   {
      behavior = statics.getBehaviorSupplier().build(new BehaviorHelper(robotModel, messager, ros2Node));
   }

   public BehaviorInterface getBuiltBehavior()
   {
      return behavior;
   }

   public BehaviorStatics getStatics()
   {
      return statics;
   }
}
