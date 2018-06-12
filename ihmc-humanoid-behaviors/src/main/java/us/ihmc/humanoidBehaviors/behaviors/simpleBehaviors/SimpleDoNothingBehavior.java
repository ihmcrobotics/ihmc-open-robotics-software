package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.Ros2Node;

public class SimpleDoNothingBehavior extends AbstractBehavior
{
   public SimpleDoNothingBehavior(String robotName, Ros2Node ros2Node)
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

   public boolean hasInputBeenSet()
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
