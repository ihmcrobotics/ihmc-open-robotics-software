package us.ihmc.behaviors.sequence;

/**
 * Base template for a robot action, like a hand pose or a walk goal.
 */
public interface BehaviorAction extends BehaviorActionData
{
   default void update()
   {

   }

   default void executeAction()
   {
      
   }

   default boolean isExecuting()
   {
      return false;
   }
}
