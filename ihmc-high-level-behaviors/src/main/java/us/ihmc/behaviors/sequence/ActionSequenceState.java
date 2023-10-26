package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionSequenceStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

import java.util.ArrayList;
import java.util.List;

public class ActionSequenceState extends BehaviorTreeNodeState<ActionSequenceDefinition>
{
   private boolean automaticExecution = false;
   private int executionNextIndex = 0;
   private ActionNodeExecutor<?, ?> lastCurrentlyExecutingAction = null;

   // This node enforces that all it's children are of a certain type
   private final List<ActionNodeState<ActionNodeDefinition>> actionChildren = new ArrayList<>();
   // TODO: Review this
   private final List<ActionNodeState<ActionNodeDefinition>> currentlyExecutingActions = new ArrayList<>();

   public ActionSequenceState(long id)
   {
      super(id, new ActionSequenceDefinition());
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
   }

   public void fromMessage(ActionSequenceStateMessage message)
   {
      getDefinition().fromMessage(message.getDefinition());

      super.fromMessage(message.getState());

      automaticExecution = message.getAutomaticExecution();
      executionNextIndex = message.getExecutionNextIndex();
   }
}
