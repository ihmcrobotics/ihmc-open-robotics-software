package us.ihmc.behaviors;

import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeControlFlowNode;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class BehaviorInterface extends BehaviorTreeControlFlowNode
{
//   private final BehaviorHelper helper;
//
//   public BehaviorInterface(BehaviorHelper helper)
//   {
//      this.helper = helper;
//   }

   public abstract void setEnabled(boolean enabled);

   public YoRegistry getYoRegistry()
   {
      return null;
   }

   public void destroy()
   {
      // TODO: Destroy behavior helper
      // helper.destroy();
   }
}
