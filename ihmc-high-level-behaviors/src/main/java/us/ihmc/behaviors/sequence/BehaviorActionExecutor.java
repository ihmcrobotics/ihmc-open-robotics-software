package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;

/**
 * Base template for a robot action, like a hand pose or a walk goal.
 */
public abstract class BehaviorActionExecutor implements BehaviorActionStateSupplier, BehaviorActionDefinitionSupplier
{
   private final BehaviorActionSequence sequence;

   public BehaviorActionExecutor(BehaviorActionSequence sequence)
   {
      this.sequence = sequence;
   }

   /** Called every tick. */
   public void update()
   {
      getState().update();
   }

   /** Trigger the action to begin executing. Called once per execution. */
   public void triggerActionExecution()
   {

   }

   /** Called every tick only when this action is executing. */
   public void updateCurrentlyExecuting()
   {

   }

   /** Should return a precalculated value from {@link #updateCurrentlyExecuting} */
   public ActionExecutionStatusMessage getExecutionStatusMessage()
   {
      return new ActionExecutionStatusMessage();
   }

   public BehaviorActionSequence getSequence()
   {
      return sequence;
   }
}
