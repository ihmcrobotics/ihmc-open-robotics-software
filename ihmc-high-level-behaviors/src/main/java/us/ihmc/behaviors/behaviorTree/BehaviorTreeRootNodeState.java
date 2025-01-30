package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage;
import gnu.trove.map.hash.TLongObjectHashMap;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.LeafNodeState;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalInteger;
import us.ihmc.communication.crdt.CRDTBidirectionalNotification;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

public class BehaviorTreeRootNodeState extends BehaviorTreeNodeState<BehaviorTreeRootNodeDefinition>
{
   private final BehaviorTreeRootNodeDefinition definition;
   private final CRDTBidirectionalBoolean automaticExecution;
   private final CRDTBidirectionalInteger executionNextIndex;
   private final CRDTBidirectionalNotification manualExecutionRequested;
   private final CRDTBidirectionalBoolean concurrencyEnabled;

   private final TLongObjectHashMap<BehaviorTreeNodeState<?>> idToNodeMap = new TLongObjectHashMap<>();
   private transient final MutableInt actionIndexAssignment = new MutableInt();
   private final List<LeafNodeState<?>> orderedLeafNodes = new ArrayList<>();
   private final List<ActionNodeState<?>> orderedActions = new ArrayList<>();

   public BehaviorTreeRootNodeState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new BehaviorTreeRootNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      definition = getDefinition();

      automaticExecution = new CRDTBidirectionalBoolean(definition, false);
      executionNextIndex = new CRDTBidirectionalInteger(definition, 0);
      manualExecutionRequested = new CRDTBidirectionalNotification(definition);
      concurrencyEnabled = new CRDTBidirectionalBoolean(definition, true);
   }

   @Override
   public void update()
   {
      super.update();

      idToNodeMap.clear();
      actionIndexAssignment.setValue(0);
      orderedLeafNodes.clear();
      updateActionSubtree(this, actionIndexAssignment);
   }

   public void updateActionSubtree(BehaviorTreeNodeState<?> node, MutableInt leafIndex)
   {
      idToNodeMap.put(node.getID(), node);

      for (BehaviorTreeNodeState<?> child : node.getChildren())
      {
         if (child instanceof LeafNodeState<?> leafNode)
         {
            leafNode.setActionIndex(leafIndex.getAndIncrement());
            orderedLeafNodes.add(leafNode);

            if (child instanceof ActionNodeState<?> action)
               orderedActions.add(action);
         }

         updateActionSubtree(child, leafIndex);
      }
   }

   public void toMessage(BehaviorTreeRootNodeStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setAutomaticExecution(automaticExecution.toMessage());
      message.setExecutionNextIndex(executionNextIndex.toMessage());
      message.setManualExecutionRequested(manualExecutionRequested.toMessage());
      message.setConcurrencyEnabled(concurrencyEnabled.toMessage());
   }

   public void fromMessage(BehaviorTreeRootNodeStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());

      automaticExecution.fromMessage(message.getAutomaticExecution());
      executionNextIndex.fromMessage(message.getExecutionNextIndex());
      manualExecutionRequested.fromMessage(message.getManualExecutionRequested());
      concurrencyEnabled.fromMessage(message.getConcurrencyEnabled());
   }

   @Nullable
   public <T extends LeafNodeState<?>> T findNextPreviousLeaf(Class<T> actionClass, int queryIndex, @Nullable RobotSide side)
   {
      T previousAction = null;
      for (int i = queryIndex - 1; i >= 0 && previousAction == null; i--)
      {
         LeafNodeState<?> leaf = orderedLeafNodes.get(i);
         if (actionClass.isInstance(leaf))
         {
            boolean match = side == null;
            match |= leaf.getDefinition() instanceof SidedObject sidedAction && sidedAction.getSide() == side;

            if (match)
            {
               previousAction = actionClass.cast(leaf);
            }
         }
      }
      return previousAction;
   }

   public void stepBackNextExecutionIndex()
   {
      if (executionNextIndex.getValue() > 0)
         executionNextIndex.decrement();
   }

   public void stepForwardNextExecutionIndex()
   {
      if (executionNextIndex.getValue() < orderedLeafNodes.size())
         executionNextIndex.increment();
   }

   public void setExecutionNextIndex(int executionNextIndex)
   {
      this.executionNextIndex.setValue(executionNextIndex);
   }

   public int getExecutionNextIndex()
   {
      return executionNextIndex.getValue();
   }

   public boolean getAutomaticExecution()
   {
      return automaticExecution.getValue();
   }

   public void setAutomaticExecution(boolean automaticExecution)
   {
      this.automaticExecution.setValue(automaticExecution);
   }

   public boolean pollManualExecutionRequested()
   {
      return manualExecutionRequested.poll();
   }

   public boolean getManualExecutionRequested()
   {
      return manualExecutionRequested.peek();
   }

   public void setManualExecutionRequested()
   {
      manualExecutionRequested.set();
   }

   public boolean getConcurrencyEnabled()
   {
      return concurrencyEnabled.getValue();
   }

   public void setConcurrencyEnabled(boolean concurrencyEnabled)
   {
      this.concurrencyEnabled.setValue(concurrencyEnabled);
   }

   public TLongObjectHashMap<BehaviorTreeNodeState<?>> getIDToNodeMap()
   {
      return idToNodeMap;
   }

   public List<LeafNodeState<?>> getOrderedLeafNodes()
   {
      return orderedLeafNodes;
   }

   public List<ActionNodeState<?>> getOrderedActions()
   {
      return orderedActions;
   }
}
