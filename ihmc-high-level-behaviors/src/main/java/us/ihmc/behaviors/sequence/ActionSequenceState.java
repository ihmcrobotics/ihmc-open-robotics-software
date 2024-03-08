package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionSequenceStateMessage;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

public class ActionSequenceState extends BehaviorTreeNodeState<ActionSequenceDefinition>
{
   private final CRDTBidirectionalBoolean automaticExecution;
   private final CRDTBidirectionalInteger executionNextIndex;
   private final CRDTUnidirectionalNotification manualExecutionRequested;
   private final CRDTUnidirectionalString nextActionRejectionTooltip;

   private transient final MutableInt actionIndex = new MutableInt();
   private final List<ActionNodeState<?>> actionChildren = new ArrayList<>();

   public ActionSequenceState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new ActionSequenceDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      automaticExecution = new CRDTBidirectionalBoolean(this, false);
      executionNextIndex = new CRDTBidirectionalInteger(this, 0);
      manualExecutionRequested = new CRDTUnidirectionalNotification(ROS2ActorDesignation.OPERATOR, crdtInfo, this);
      nextActionRejectionTooltip = new CRDTUnidirectionalString(ROS2ActorDesignation.ROBOT, crdtInfo, "");
   }

   @Override
   public void update()
   {
      super.update();

      actionIndex.setValue(0);
      actionChildren.clear();
      updateActionSubtree(this, actionIndex);

      updateExecuteAfterNodeReferences();
   }

   public void updateExecuteAfterNodeReferences()
   {
      for (int i = 0; i < actionChildren.size(); i++)
      {
         ActionNodeState<?> actionState = actionChildren.get(i);
         ActionNodeState<?> executeAfterNode = actionState.getExecuteAfterNode();

         // Remove non-existent nodes from being referenced
         if (executeAfterNode != null && !actionChildren.contains(executeAfterNode))
            actionState.setExecuteAfterNode(null);

         if (executeAfterNode != null)
         {
            if (actionState.getDefinition().getExecuteAfterBeginning())
               actionState.setExecuteAfterNode(null);

            // Remove reference to previous node if it's no longer immediately previous
            if (actionState.getDefinition().getExecuteAfterPrevious() && executeAfterNode.getActionIndex() != actionState.getActionIndex() - 1)
               actionState.setExecuteAfterNode(null);

            // If the name in the definition doesn't match the name of the referenced action we need to research
            if (!actionState.getDefinition().getExecuteAfterBeginning()
             && !actionState.getDefinition().getExecuteAfterPrevious()
             && !executeAfterNode.getDefinition().getName().equals(actionState.getDefinition().getExecuteAfterAction()))
               actionState.setExecuteAfterNode(null);
         }

         // Search backward for matching node only if we don't have it already
         if (executeAfterNode == null)
         {
            for (int j = i - 1; j >= 0; j--)
            {
               if (actionState.getDefinition().getExecuteAfterPrevious() // Will break on the first iteration in this case
                || actionState.getDefinition().getExecuteAfterAction().equals(actionChildren.get(j).getDefinition().getName()))
               {
                  actionChildren.get(i).setExecuteAfterNode(actionChildren.get(j));
                  break;
               }
            }
         }
      }
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

   public void toMessage(ActionSequenceStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setAutomaticExecution(automaticExecution.toMessage());
      message.setExecutionNextIndex(executionNextIndex.toMessage());
      message.setManualExecutionRequested(manualExecutionRequested.toMessage());
      message.setNextActionRejectionTooltip(nextActionRejectionTooltip.toMessage());
   }

   public void fromMessage(ActionSequenceStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      automaticExecution.fromMessage(message.getAutomaticExecution());
      executionNextIndex.fromMessage(message.getExecutionNextIndex());
      manualExecutionRequested.fromMessage(message.getManualExecutionRequested());
      nextActionRejectionTooltip.fromMessage(message.getNextActionRejectionTooltipAsString());
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

   public void setNextActionRejectionTooltip(String nextActionRejectionTooltip)
   {
      this.nextActionRejectionTooltip.setValue(nextActionRejectionTooltip);
   }

   public String getNextActionRejectionTooltip()
   {
      return nextActionRejectionTooltip.getValue();
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

   public List<ActionNodeState<?>> getActionChildren()
   {
      return actionChildren;
   }
}
