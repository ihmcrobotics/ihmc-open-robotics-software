package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import us.ihmc.tools.string.StringTools;

/**
 * Base template for a robot action, like a hand pose or a walk goal.
 */
public interface BehaviorActionExecutor extends BehaviorActionStateSupplier, BehaviorActionDefinitionSupplier
{
   /** Called every tick. */
   void update(int actionIndex, int nextExecutionIndex, boolean concurrentActionIsNextForExecution);

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

      if (this instanceof FrameBasedBehaviorActionDefinition frameBasedBehaviorActionDefinition)
      {
         canExecute &= frameBasedBehaviorActionDefinition.getConditionalReferenceFrame().hasParentFrame();
      }

      // TODO: add other conditions

      return canExecute;
   }

   default StringBuilder getExecutionRejectionTooltip()
   {
      StringBuilder tooltip = new StringBuilder();

      if (this instanceof FrameBasedBehaviorActionDefinition frameBasedBehaviorActionDefinition)
      {
         if (!frameBasedBehaviorActionDefinition.getConditionalReferenceFrame().hasParentFrame())
         {
            tooltip.append(StringTools.format("Parent frame [{}] does not exist in the scene",
                                              frameBasedBehaviorActionDefinition.getConditionalReferenceFrame().getConditionallyValidParentFrameName()).get());
         }
      }

      // TODO: add other conditions

      return tooltip;
   }
}
