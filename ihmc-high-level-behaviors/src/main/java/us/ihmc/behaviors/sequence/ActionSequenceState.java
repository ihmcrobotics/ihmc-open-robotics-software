package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionSequenceStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTNotification;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

import java.util.ArrayList;
import java.util.List;

public class ActionSequenceState extends BehaviorTreeNodeState<ActionSequenceDefinition>
{
   private boolean automaticExecution = false;
   private int executionNextIndex = 0;
   private ActionNodeExecutor<?, ?> lastCurrentlyExecutingAction = null;
   private String nextActionRejectionTooltip = "";
   private CRDTNotification manualExecutionRequested;

   // This node enforces that all it's children are of a certain type
   private final List<ActionNodeState<ActionNodeDefinition>> actionChildren = new ArrayList<>();
   // TODO: Review this
   private final List<ActionNodeState<ActionNodeDefinition>> currentlyExecutingActions = new ArrayList<>();

   public ActionSequenceState(long id, ROS2ActorDesignation actorDesignation)
   {
      super(id, new ActionSequenceDefinition());

      manualExecutionRequested = new CRDTNotification(ROS2ActorDesignation.OPERATOR, actorDesignation);
   }

   @Override
   public void update()
   {
      actionChildren.clear();
      for (BehaviorTreeNodeState<?> child : getChildren())
      {
         actionChildren.add((ActionNodeState<ActionNodeDefinition>) child);
      }

      for (int i = 0; i < getChildren().size(); i++)
      {
         BehaviorActionExecutionStatusCalculator.update(actionChildren, i, executionNextIndex);
      }

   }

   private boolean noCurrentActionIsExecuting()
   {
      boolean noCurrentActionIsExecuting = true;
      for (ActionNodeState currentlyExecutingAction : currentlyExecutingActions)
      {
         noCurrentActionIsExecuting &= !currentlyExecutingAction.getIsExecuting();
      }
      return noCurrentActionIsExecuting;
   }

   // TODO: Where to put this
   public void onExecutionIndexChanged()
   {
      lastCurrentlyExecutingAction = null; // Set to null so we don't wait for that action to complete
      currentlyExecutingActions.clear();
   }

   public void toMessage(ActionSequenceStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setAutomaticExecution(automaticExecution);
      message.setExecutionNextIndex(executionNextIndex);
      message.setNextActionRejectionTooltip(nextActionRejectionTooltip);

      manualExecutionRequested.toMessage(message.getManualExecutionRequested());
   }

   public void fromMessage(ActionSequenceStateMessage message)
   {
      getDefinition().fromMessage(message.getDefinition());

      super.fromMessage(message.getState());

      // Only modified by robot
      nextActionRejectionTooltip = message.getNextActionRejectionTooltipAsString();

      // Only modified by operator

      if (isFrozenFromModification()) // Can be modified by either
      {
         automaticExecution = message.getAutomaticExecution();
      }

      // Special cases
      manualExecutionRequested.fromMessage(message.getManualExecutionRequested());
      executionNextIndex = message.getExecutionNextIndex();
   }

   public void stepBackNextExecutionIndex()
   {
      if (executionNextIndex > 0)
         --executionNextIndex;
   }

   public void stepForwardNextExecutionIndex()
   {
      if (executionNextIndex < getChildren().size())
         ++executionNextIndex;
   }

   public void setExecutionNextIndex(int executionNextIndex)
   {
      this.executionNextIndex = executionNextIndex;
   }

   public int getExecutionNextIndex()
   {
      return executionNextIndex;
   }

   public void setNextActionRejectionTooltip(String nextActionRejectionTooltip)
   {
      this.nextActionRejectionTooltip = nextActionRejectionTooltip;
   }

   public String getNextActionRejectionTooltip()
   {
      return nextActionRejectionTooltip;
   }

   public boolean getAutomaticExecution()
   {
      return automaticExecution;
   }

   public void setAutomaticExecution(boolean automaticExecution)
   {
      this.automaticExecution = automaticExecution;
   }

   public CRDTNotification getManualExecutionRequested()
   {
      return manualExecutionRequested;
   }

   public List<ActionNodeState<ActionNodeDefinition>> getCurrentlyExecutingActions()
   {
      return currentlyExecutingActions;
   }
}
