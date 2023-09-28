package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;

/**
 * Base template for a robot action, like a hand pose or a walk goal.
 */
public interface BehaviorAction extends BehaviorActionDescription
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

      if (this instanceof FrameBasedBehaviorActionDescription frameBasedBehaviorActionDescription)
      {
         canExecute &= frameBasedBehaviorActionDescription.getConditionalReferenceFrame().hasParentFrame();
      }

      // TODO: add other conditions

      return canExecute;
   }

   default StringBuilder getExecutionRejectionTooltip()
   {
      StringBuilder tooltip = new StringBuilder();

      if (this instanceof FrameBasedBehaviorActionDescription frameBasedBehaviorActionDescription)
      {
         if (!frameBasedBehaviorActionDescription.getConditionalReferenceFrame().hasParentFrame())
         {
            tooltip.append("parent frame does not exist in the scene");
         }
      }

      // TODO: add other conditions

      return tooltip;
   }
}
