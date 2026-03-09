package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage;
import gnu.trove.map.hash.TLongObjectHashMap;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
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
   private final CRDTBidirectionalBoolean automaticExecution;
   private final CRDTBidirectionalInteger executionNextIndex;
   private final CRDTBidirectionalNotification manualExecutionRequested;
   private final CRDTBidirectionalNotification failureResetRequested;
   private final CRDTBidirectionalBoolean concurrencyEnabled;
   private final CRDTBidirectionalBoolean previewModeEnabled;

   private final TLongObjectHashMap<BehaviorTreeNodeState<?>> idToNodeMap = new TLongObjectHashMap<>();
   private transient final MutableInt depthFirstIndexAssignment = new MutableInt();
   private transient final MutableInt leafIndexAssignment = new MutableInt();
   private final List<BehaviorTreeNodeState<?>> orderedNodes = new ArrayList<>();
   private final List<LeafNodeState<?>> orderedLeaves = new ArrayList<>();
   private final List<ActionNodeState<?>> orderedActions = new ArrayList<>();

   public BehaviorTreeRootNodeState(long id,
                                    CRDTInfo crdtInfo,
                                    WorkspaceResourceDirectory saveFileDirectory,
                                    DRCRobotModel robotModel,
                                    BehaviorTreeSceneState scene)
   {
      super(id, new BehaviorTreeRootNodeDefinition(crdtInfo, saveFileDirectory, robotModel), null, scene);

      automaticExecution = new CRDTBidirectionalBoolean(definition, false);
      executionNextIndex = new CRDTBidirectionalInteger(definition, 0);
      manualExecutionRequested = new CRDTBidirectionalNotification(definition);
      concurrencyEnabled = new CRDTBidirectionalBoolean(definition, true);
      previewModeEnabled = new CRDTBidirectionalBoolean(definition, false);
      failureResetRequested = new CRDTBidirectionalNotification(definition);
   }

   @Override
   public void update()
   {
      super.update();

      idToNodeMap.clear();
      depthFirstIndexAssignment.setValue(0);
      leafIndexAssignment.setValue(0);
      orderedNodes.clear();
      orderedLeaves.clear();
      orderedActions.clear();
      updateSubtree(this, depthFirstIndexAssignment, leafIndexAssignment);
   }

   public void updateSubtree(BehaviorTreeNodeState<?> node, MutableInt depthFirstIndex, MutableInt leafIndex)
   {
      idToNodeMap.put(node.getID(), node);
      node.setDepthFirstIndex(depthFirstIndex.getAndIncrement());
      orderedNodes.add(node);

      for (BehaviorTreeNodeState<?> child : node.getChildren())
      {
         if (child instanceof LeafNodeState<?> leafNode)
         {
            leafNode.setLeafIndex(leafIndex.getAndIncrement());
            orderedLeaves.add(leafNode);

            if (child instanceof ActionNodeState<?> action)
               orderedActions.add(action);
         }

         updateSubtree(child, depthFirstIndex, leafIndex);
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
      message.setPreviewModeEnabled(previewModeEnabled.toMessage());
      message.setFailureResetRequested(failureResetRequested.toMessage());
   }

   public void fromMessage(BehaviorTreeRootNodeStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());

      automaticExecution.fromMessage(message.getAutomaticExecution());
      executionNextIndex.fromMessage(message.getExecutionNextIndex());
      manualExecutionRequested.fromMessage(message.getManualExecutionRequested());
      concurrencyEnabled.fromMessage(message.getConcurrencyEnabled());
      previewModeEnabled.fromMessage(message.getPreviewModeEnabled());
      failureResetRequested.fromMessage(message.getFailureResetRequested());
   }

   @Nullable
   public <T extends LeafNodeState<?>> T findNextPreviousLeaf(Class<T> leafClass, int queryIndex, @Nullable RobotSide side)
   {
      T previousLeaf = null;
      for (int i = queryIndex - 1; orderedLeaves.size() > i && i >= 0 && previousLeaf == null; i--)
      {
         LeafNodeState<?> leaf = orderedLeaves.get(i);
         if (leafClass.isInstance(leaf))
         {
            boolean match = side == null;
            match |= leaf.getDefinition() instanceof SidedObject sidedAction && sidedAction.getSide() == side;

            if (match)
            {
               previousLeaf = leafClass.cast(leaf);
            }
         }
      }
      return previousLeaf;
   }

   public void stepBackNextExecutionIndex()
   {
      if (executionNextIndex.getValue() > 0)
         executionNextIndex.decrement();
   }

   public void stepForwardNextExecutionIndex()
   {
      if (executionNextIndex.getValue() < orderedLeaves.size())
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

   public boolean pollFailureResetRequested()
   {
      return failureResetRequested.poll();
   }

   public boolean getFailureResetRequested()
   {
      return failureResetRequested.peek();
   }

   public void setFailureResetRequested()
   {
      failureResetRequested.set();
   }

   public boolean getConcurrencyEnabled()
   {
      return concurrencyEnabled.getValue();
   }

   public void setConcurrencyEnabled(boolean concurrencyEnabled)
   {
      this.concurrencyEnabled.setValue(concurrencyEnabled);
   }

   public boolean getPreviewModeEnabled()
   {
      return previewModeEnabled.getValue();
   }

   public void setPreviewModeEnabled(boolean previewModeEnabled)
   {
      this.previewModeEnabled.setValue(previewModeEnabled);
   }

   public TLongObjectHashMap<BehaviorTreeNodeState<?>> getIDToNodeMap()
   {
      return idToNodeMap;
   }

   public List<BehaviorTreeNodeState<?>> getOrderedNodes()
   {
      return orderedNodes;
   }

   public List<LeafNodeState<?>> getOrderedLeaves()
   {
      return orderedLeaves;
   }

   public List<ActionNodeState<?>> getOrderedActions()
   {
      return orderedActions;
   }

   // Getters are in here so there's not getters in base node for root stuff

   public BehaviorTreeSceneState getScene()
   {
      return scene;
   }
}
