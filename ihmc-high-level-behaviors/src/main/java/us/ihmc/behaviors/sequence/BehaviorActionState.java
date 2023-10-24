package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.BehaviorActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public abstract class BehaviorActionState<D extends BehaviorActionDefinition>
      extends BehaviorTreeNodeState<D>
{
   private int actionIndex = -1;
   private boolean isNextForExecution = false;
   private boolean isToBeExecutedConcurrently = false;
   private boolean canExecute = true;
   private boolean isExecuting = false;

   public BehaviorActionState(long id, D definition)
   {
      super(id, definition);
   }

   public void toMessage(BehaviorActionStateMessage message)
   {
      super.toMessage(message.getNodeState());

      message.setActionIndex(actionIndex);
      message.setIsNextForExecution(isNextForExecution);
      message.setIsToBeExecutedConcurrently(isToBeExecutedConcurrently);
      message.setCanExecute(canExecute);
      message.setIsExecuting(isExecuting);
   }

   public void fromMessage(BehaviorActionStateMessage message)
   {
      super.fromMessage(message.getNodeState());

      actionIndex = message.getActionIndex();
      isNextForExecution = message.getIsNextForExecution();
      isToBeExecutedConcurrently = message.getIsToBeExecutedConcurrently();
      canExecute = message.getCanExecute();
      isExecuting = message.getIsExecuting();
   }

   public void setActionIndex(int actionIndex)
   {
      this.actionIndex = actionIndex;
   }

   public int getActionIndex()
   {
      return actionIndex;
   }

   public void setIsNextForExecution(boolean isNextForExecution)
   {
      this.isNextForExecution = isNextForExecution;
   }

   public boolean getIsNextForExecution()
   {
      return isNextForExecution;
   }

   public void setIsToBeExecutedConcurrently(boolean isToBeExecutedConcurrently)
   {
      this.isToBeExecutedConcurrently = isToBeExecutedConcurrently;
   }

   public boolean getIsToBeExecutedConcurrently()
   {
      return isToBeExecutedConcurrently;
   }

   public void setCanExecute(boolean canExecute)
   {
      this.canExecute = canExecute;
   }

   public boolean getCanExecute()
   {
      return canExecute;
   }

   public void setIsExecuting(boolean isExecuting)
   {
      this.isExecuting = isExecuting;
   }

   /** Should return a precalculated value from {@link BehaviorActionExecutor#updateCurrentlyExecuting} */
   public boolean getIsExecuting()
   {
      return isExecuting;
   }
}
