package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage;

import java.util.ArrayList;
import java.util.List;

/**
 * The core interface of a Behavior Tree: the node that can be ticked.
 */
public abstract class BehaviorTreeNodeState implements BehaviorTreeNodeDefinitionSupplier
{
   /**
    * A node is active if it lies on the path of the current tree tick.
    *
    * Having this property being a part of every node in the tree enables any
    * node to know if it is no longer on the path of the current tick and can
    * take action based on that which is typically maintenance of it's threads
    * and disabling active elements automatically.
    */
   private boolean isActive = false;

   private final List<BehaviorTreeNodeState> children = new ArrayList<>();


   public void toMessage(BehaviorTreeNodeStateMessage message)
   {
      message.setIsActive(isActive);
   }

   public void fromMessage(BehaviorTreeNodeStateMessage message)
   {
      isActive = message.getIsActive();
   }

   public void setIsActive(boolean isActive)
   {
      this.isActive = isActive;
   }

   public boolean getIsActive()
   {
      return isActive;
   }

   public List<BehaviorTreeNodeState> getChildren()
   {
      return children;
   }
}
