package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import us.ihmc.tools.string.StringTools;

/**
 * Base template for a robot action, like a hand pose or a walk goal.
 */
public interface BehaviorActionExecutor extends BehaviorActionDefinition
{
   /** Called every tick. */
   default void update(int actionIndex, int nextExecutionIndex, boolean concurrentActionIsNextForExecution)
   {
      update();
   }

   /** Trigger the action to begin executing. Called once per execution. */
   default void triggerActionExecution()
   {

   }

   /** Called every tick only when this action is executing. */
   default void updateCurrentlyExecuting()
   {

   }

   /** Should return a precalculated value from {@link #updateCurrentlyExecuting} */
   default ActionExecutionStatusMessage getExecutionStatusMessage()
   {
      return new ActionExecutionStatusMessage();
   }

   /** Should return a precalculated value from {@link #updateCurrentlyExecuting} */
   default boolean isExecuting()
   {
      return false;
   }

   default boolean canExecute()
   {
      boolean canExecute = true;

      // TODO: add other conditions

      return canExecute;
   }
}
