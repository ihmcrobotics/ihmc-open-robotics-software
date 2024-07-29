package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalInteger;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTUnidirectionalNotification;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
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
   private final CRDTUnidirectionalNotification manualExecutionRequested;
   private final CRDTBidirectionalBoolean concurrencyEnabled;

   private transient final MutableInt actionIndex = new MutableInt();
   private final List<ActionNodeState<?>> actionChildren = new ArrayList<>();

   public BehaviorTreeRootNodeState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new BehaviorTreeRootNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      definition = getDefinition();

      automaticExecution = new CRDTBidirectionalBoolean(definition, false);
      executionNextIndex = new CRDTBidirectionalInteger(definition, 0);
      manualExecutionRequested = new CRDTUnidirectionalNotification(ROS2ActorDesignation.OPERATOR, definition);
      concurrencyEnabled = new CRDTBidirectionalBoolean(definition, true);
   }

   @Override
   public void update()
   {
      super.update();

      actionIndex.setValue(0);
      actionChildren.clear();
      updateActionSubtree(this, actionIndex);
   }

   public void updateActionSubtree(BehaviorTreeNodeState<?> node, MutableInt actionIndex)
   {
      for (BehaviorTreeNodeState<?> child : node.getChildren())
      {
         if (child instanceof ActionNodeState<?> actionNode)
         {
            actionNode.setActionIndex(actionIndex.getAndIncrement());
            actionChildren.add(actionNode);
         }
         else
         {
            updateActionSubtree(child, actionIndex);
         }
      }
   }

   public void toMessage(BehaviorTreeRootNodeStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setAutomaticExecution(automaticExecution.toMessage());
      message.setExecutionNextIndex(executionNextIndex.toMessage());
      message.setManualExecutionRequested(manualExecutionRequested.toMessage());
      message.setConcurrencyEnabled(concurrencyEnabled.toMessage());
   }

   public void fromMessage(BehaviorTreeRootNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      automaticExecution.fromMessage(message.getAutomaticExecution());
      executionNextIndex.fromMessage(message.getExecutionNextIndex());
      manualExecutionRequested.fromMessage(message.getManualExecutionRequested());
      concurrencyEnabled.fromMessage(message.getConcurrencyEnabled());
   }

   @Nullable
   public <T extends ActionNodeState<?>> T findNextPreviousAction(Class<T> actionClass, int queryIndex, @Nullable RobotSide side)
   {
      T previousAction = null;
      for (int i = queryIndex - 1; i >= 0 && previousAction == null; i--)
      {
         ActionNodeState<?> action = actionChildren.get(i);
         if (actionClass.isInstance(action))
         {
            boolean match = side == null;
            match |= action.getDefinition() instanceof SidedObject sidedAction && sidedAction.getSide() == side;

            if (match)
            {
               previousAction = actionClass.cast(action);
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
      if (executionNextIndex.getValue() < actionChildren.size())
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

   public List<ActionNodeState<?>> getActionChildren()
   {
      return actionChildren;
   }
}
