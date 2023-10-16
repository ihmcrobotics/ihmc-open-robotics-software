package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.BehaviorActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

public class BehaviorActionState extends BehaviorTreeNodeState implements BehaviorActionDefinitionSupplier
{
   private final BehaviorActionDefinition definition;

   /** The action's unique ID. */
   private long id;
   private int actionIndex = -1;
   private boolean isNextForExecution = false;
   private boolean isToBeExecutedConcurrently = false;
   private boolean canExecute = true;
   private boolean isExecuting = false;

   /**
    * For when nothing extends this node type.
    */
   public BehaviorActionState()
   {
      this(new BehaviorActionDefinition());
   }

   /**
    * For use by a node extending this one.
    */
   public BehaviorActionState(BehaviorActionDefinition definition)
   {
      // The definition in this class extends the definition in the superclass
      // and is the same instance.
      super(definition);
      this.definition = definition;

      // TODO: Re-enable when we do the CRDT
//      id = BehaviorActionSequence.NEXT_ID.getAndIncrement();
   }

   public void update()
   {

   }

   public void toMessage(BehaviorActionStateMessage message)
   {
      if (definition.getClass() == BehaviorActionDefinition.class)
         definition.toMessage(message.getDefinition());

      message.setId(id);
      message.setActionIndex(actionIndex);
      message.setIsNextForExecution(isNextForExecution);
      message.setIsToBeExecutedConcurrently(isToBeExecutedConcurrently);
   }

   public void fromMessage(BehaviorActionStateMessage message)
   {
//      if (id != message.getId())
//         LogTools.error("IDs should match!");

      id = message.getId();
      actionIndex = message.getActionIndex();
      isNextForExecution = message.getIsNextForExecution();
      isToBeExecutedConcurrently = message.getIsToBeExecutedConcurrently();
   }

   /** The action's unique ID. */
   public long getID()
   {
      return id;
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
