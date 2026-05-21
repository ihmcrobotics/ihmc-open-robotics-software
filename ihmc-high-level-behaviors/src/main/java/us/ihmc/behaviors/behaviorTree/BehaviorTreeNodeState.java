package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.BehaviorTreeNodeStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.behaviorTree.log.BehaviorTreeNodeMessageLogger;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

/**
 * A behavior tree node layer that sits over the Definition layer.
 * The state layer is the layer that gets synchronized over the network.
 *
 * @param <D> The type of this node's definition instance.
 */
public class BehaviorTreeNodeState<D extends BehaviorTreeNodeDefinition> implements TreeNode<BehaviorTreeNodeState<?>>
{
   /** Convenient accessor to the definition to keep the code clean, available to all inheriting classes. */
   protected final D definition;

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

   /** The index is not CRDT synced because it's a simple local calculation. */
   protected int depthFirstIndex = -1;

   /**
    * The state's children. They can be any type that is a BehaviorTreeNodeState.
    */
   private final List<BehaviorTreeNodeState<?>> children = new ArrayList<>();
   protected final BehaviorTreeRootNodeState rootNode;
   private transient BehaviorTreeNodeState<?> parent;

   private final BehaviorTreeNodeMessageLogger logger;
   protected final CRDTInfo crdtInfo; // convenient to have readily available
   protected final BehaviorTreeSceneState scene;
   protected final DRCRobotModel robotModel;

   public BehaviorTreeNodeState(long id, D definition, BehaviorTreeRootNodeState rootNode)
   {
      this(id, definition, rootNode, rootNode.getScene());
   }

   public BehaviorTreeNodeState(long id, D definition, BehaviorTreeRootNodeState rootNode, BehaviorTreeSceneState scene)
   {
      this.id = id;
      this.definition = definition;
      this.crdtInfo = definition.getCRDTInfo();
      if (rootNode == null)
      {
         this.rootNode = (BehaviorTreeRootNodeState) this;
         this.robotModel = ((BehaviorTreeRootNodeDefinition) definition).getRobotModel();
      }
      else
      {
         this.rootNode = rootNode;
         this.robotModel = rootNode.getDefinition().getRobotModel();
      }
      this.scene = scene;

      logger = new BehaviorTreeNodeMessageLogger(definition.getCRDTInfo());
   }

   /** Used to determine if the node's full data needs to be sent. */
   public boolean hasStatus()
   {
      return false;
   }

   public void toMessage(BehaviorTreeNodeStateMessage message)
   {
      message.setId((int) id);
      message.setIsActive(isActive);

      logger.toMessage(message.getRecentLogMessages());
   }

   public void fromMessage(BehaviorTreeNodeStateMessage message)
   {
      if (id != message.getId())
         LogTools.error(("IDs should match! %s:%d != message.id: %d").formatted(definition.getName(), id, message.getId()));

      isActive = message.getIsActive();

      logger.fromMessage(message.getRecentLogMessages());
   }

   /** Update the node's state. Should not have side effects if called multiple times per tick. */
   public void update()
   {
      definition.checkModified();
      definition.updateName();
   }

   public void drawToSVG()
   {
      new BehaviorTreeSVGWriter(this);
   }

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

   public void setDepthFirstIndex(int depthFirstIndex)
   {
      this.depthFirstIndex = depthFirstIndex;
   }

   /** The index of the node, depth first over the entire tree. The root is 0. */
   public int getDepthFirstIndex()
   {
      return depthFirstIndex;
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

   public D getDefinition()
   {
      return definition;
   }

   public BehaviorTreeNodeMessageLogger getLogger()
   {
      return logger;
   }
}
