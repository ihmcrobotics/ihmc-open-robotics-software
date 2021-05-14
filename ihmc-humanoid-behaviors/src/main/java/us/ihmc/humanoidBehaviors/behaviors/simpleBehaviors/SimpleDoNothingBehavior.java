package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.ROS2Node;

public class SimpleDoNothingBehavior extends AbstractBehavior
{
   public SimpleDoNothingBehavior(String robotName, ROS2Node ros2Node)
   {
      super(robotName, ros2Node);
   }

   @Override
   public void doControl()
   {
   }

   @Override
   public boolean isDone()
   {
      return true;
   }

   @Override
   public void onBehaviorEntered()
   {
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

   @Override
   public void onBehaviorExited()
   {
   }
}
