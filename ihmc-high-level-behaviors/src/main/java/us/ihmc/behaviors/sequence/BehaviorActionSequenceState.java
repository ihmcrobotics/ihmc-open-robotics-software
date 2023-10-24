package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

import java.util.ArrayList;
import java.util.List;

public class BehaviorActionSequenceState extends BehaviorTreeNodeState<BehaviorActionSequenceDefinition>
{
   private boolean automaticExecution = false;
   private int executionNextIndex = 0;
   private BehaviorActionExecutor<?, ?> lastCurrentlyExecutingAction = null;

   // This node enforces that all it's children are of a certain type
   private final List<BehaviorActionState<BehaviorActionDefinition>> actionChildren = new ArrayList<>();
   // TODO: Review this
   private final List<BehaviorActionState<BehaviorActionDefinition>> currentlyExecutingActions = new ArrayList<>();

   public BehaviorActionSequenceState(long id)
   {
      super(id, new BehaviorActionSequenceDefinition());
   }

   @Override
   public void update()
   {
      actionChildren.clear();
      for (BehaviorTreeNodeState<?> child : getChildren())
      {
         actionChildren.add((BehaviorActionState<BehaviorActionDefinition>) child);
      }

      for (int i = 0; i < getChildren().size(); i++)
      {
         BehaviorActionExecutionStatusCalculator.update(actionChildren, i, executionNextIndex);
      }

   }

   private boolean noCurrentActionIsExecuting()
   {
      boolean noCurrentActionIsExecuting = true;
      for (BehaviorActionState currentlyExecutingAction : currentlyExecutingActions)
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

   public void toMessage(BehaviorActionSequenceStateMessage message)
   {
      super.toMessage(message.getState());

      message.setAutomaticExecution(automaticExecution);
      message.setExecutionNextIndex(executionNextIndex);
   }

   public void fromMessage(BehaviorActionSequenceStateMessage message)
   {
      super.fromMessage(message.getState());

      automaticExecution = message.getAutomaticExecution();
      executionNextIndex = message.getExecutionNextIndex();
   }
}
