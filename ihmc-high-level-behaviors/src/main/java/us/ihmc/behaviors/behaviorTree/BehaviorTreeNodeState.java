package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.log.BehaviorTreeNodeMessageLogger;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

/**
 * A behavior tree node layer that sits over the Definition layer.
 * The state layer is the layer that gets synchronized over the network.
 */
public class BehaviorTreeNodeState<D extends BehaviorTreeNodeDefinition>
      implements BehaviorTreeNodeLayer<BehaviorTreeNodeState<?>, D, BehaviorTreeNodeState<D>, D>
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
   private transient BehaviorTreeNodeState<?> parent;

   private final BehaviorTreeNodeMessageLogger logger;

   public BehaviorTreeNodeState(long id, D definition, CRDTInfo crdtInfo)
   {
      this.id = id;
      this.definition = definition;

      logger = new BehaviorTreeNodeMessageLogger(definition);
   }

   public void toMessage(BehaviorTreeNodeStateMessage message)
   {
      message.setId(id);
      message.setIsActive(isActive);

      logger.toMessage(message.getRecentLogMessages());
   }

   public void fromMessage(BehaviorTreeNodeStateMessage message)
   {
      if (id != message.getId())
         LogTools.error("IDs should match! {} != {}", id, message.getId());

      isActive = message.getIsActive();

      logger.fromMessage(message.getRecentLogMessages());
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
   public void setParent(@Nullable BehaviorTreeNodeState<?> parent)
   {
      this.parent = parent;
   }

   @Nullable
   @Override
   public BehaviorTreeNodeState<?> getParent()
   {
      return parent;
   }

   @Override
   public D getNextLowerLayer()
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

   public BehaviorTreeNodeMessageLogger getLogger()
   {
      return logger;
   }
}
