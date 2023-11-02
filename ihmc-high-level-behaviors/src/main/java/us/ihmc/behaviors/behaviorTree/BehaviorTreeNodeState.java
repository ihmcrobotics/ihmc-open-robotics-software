package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.Confirmable;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.List;

/**
 * The core interface of a Behavior Tree: the node that can be ticked.
 */
public abstract class BehaviorTreeNodeState<D extends BehaviorTreeNodeDefinition>
      extends Confirmable
      implements BehaviorTreeNodeExtension<BehaviorTreeNodeState<?>, D, BehaviorTreeNodeState<D>, D>
{
   private final D definition;

   /** The node's unique ID. */
   private final long id;
   /**
    * A node is active if it lies on the path of the current tree tick.
    *
    * Having this property being a part of every node in the tree enables any
    * node to know if it is no longer on the path of the current tick and can
    * take action based on that which is typically maintenance of it's threads
    * and disabling active elements automatically.
    */
   private boolean isActive = false;

   /**
    * The state's children. They can be any type that is a BehaviorTreeNodeState.
    */
   private final List<BehaviorTreeNodeState<?>> children = new ArrayList<>();

   public BehaviorTreeNodeState(long id, D definition, CRDTInfo crdtInfo)
   {
      super(crdtInfo);

      this.id = id;
      this.definition = definition;
   }

   public void toMessage(BehaviorTreeNodeStateMessage message)
   {
      toMessage(message.getConfirmableRequest());

      message.setId(id);
      message.setIsActive(isActive);
   }

   public void fromMessage(BehaviorTreeNodeStateMessage message)
   {
      fromMessage(message.getConfirmableRequest()); // Unpack first, because this also unfreezes

      if (id != message.getId())
         LogTools.error("IDs should match! {} != {}", id, message.getId());

      isActive = message.getIsActive();
   }

   @Override
   public void destroy()
   {

   }

   /** The node's unique ID. */
   public long getID()
   {
      return id;
   }

   public void setIsActive(boolean isActive)
   {
      this.isActive = isActive;
   }

   public boolean getIsActive()
   {
      return isActive;
   }

   @Override
   public List<BehaviorTreeNodeState<?>> getChildren()
   {
      return children;
   }

   @Override
   public D getExtendedNode()
   {
      return getDefinition();
   }

   @Override
   public D getDefinition()
   {
      return definition;
   }

   @Override
   public BehaviorTreeNodeState<D> getState()
   {
      return this;
   }
}
