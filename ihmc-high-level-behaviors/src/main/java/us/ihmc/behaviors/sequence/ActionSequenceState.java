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
   private final CRDTUnidirectionalBoolean concurrencyEnabled;
   private final CRDTUnidirectionalString nextActionRejectionTooltip;

   private transient final MutableInt actionIndex = new MutableInt();
   private final List<ActionNodeState<?>> actionChildren = new ArrayList<>();

   public ActionSequenceState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new ActionSequenceDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      automaticExecution = new CRDTBidirectionalBoolean(this, false);
      executionNextIndex = new CRDTBidirectionalInteger(this, 0);
      manualExecutionRequested = new CRDTUnidirectionalNotification(ROS2ActorDesignation.OPERATOR, crdtInfo, this);
      concurrencyEnabled = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, false);
      nextActionRejectionTooltip = new CRDTUnidirectionalString(ROS2ActorDesignation.ROBOT, crdtInfo, "");
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

   public void toMessage(ActionSequenceStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setAutomaticExecution(automaticExecution.toMessage());
      message.setExecutionNextIndex(executionNextIndex.toMessage());
      message.setManualExecutionRequested(manualExecutionRequested.toMessage());
      message.setConcurrencyEnabled(concurrencyEnabled.toMessage());
      message.setNextActionRejectionTooltip(nextActionRejectionTooltip.toMessage());
   }

   public void fromMessage(ActionSequenceStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      automaticExecution.fromMessage(message.getAutomaticExecution());
      executionNextIndex.fromMessage(message.getExecutionNextIndex());
      manualExecutionRequested.fromMessage(message.getManualExecutionRequested());
      concurrencyEnabled.fromMessage(message.getConcurrencyEnabled());
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
