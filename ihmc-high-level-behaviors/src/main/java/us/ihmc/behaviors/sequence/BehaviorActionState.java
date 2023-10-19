package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.BehaviorActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public abstract class BehaviorActionState extends BehaviorTreeNodeState implements BehaviorActionDefinitionSupplier
{
   private final BehaviorActionDefinition definition;

   private int actionIndex = -1;
   private boolean isNextForExecution = false;
   private boolean isToBeExecutedConcurrently = false;
   private boolean canExecute = true;
   private boolean isExecuting = false;

   public BehaviorActionState(long id, BehaviorActionDefinition definition)
   {
      super(id, definition);

      this.definition = definition;
   }

   public void update()
   {

   }

   public void toMessage(BehaviorActionStateMessage message)
   {
      super.toMessage(message.getNodeState());

      message.setActionIndex(actionIndex);
      message.setIsNextForExecution(isNextForExecution);
      message.setIsToBeExecutedConcurrently(isToBeExecutedConcurrently);
   }

   public void fromMessage(BehaviorActionStateMessage message)
   {
      super.fromMessage(message.getNodeState());

      actionIndex = message.getActionIndex();
      isNextForExecution = message.getIsNextForExecution();
      isToBeExecutedConcurrently = message.getIsToBeExecutedConcurrently();
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

   @Override
   public BehaviorActionDefinition getDefinition()
   {
      return definition;
   }
}
