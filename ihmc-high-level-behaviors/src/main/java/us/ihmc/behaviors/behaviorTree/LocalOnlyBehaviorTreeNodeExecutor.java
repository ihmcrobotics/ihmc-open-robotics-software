package us.ihmc.behaviors.behaviorTree;

/**
 * This is currently around to keep older behavior tree nodes
 * compiling without them fully adhering to the newer standards.
 */
public class LocalOnlyBehaviorTreeNodeExecutor extends BehaviorTreeNodeExecutor
{
   private final LocalOnlyBehaviorTreeNodeState state = new LocalOnlyBehaviorTreeNodeState();

   @Override
   public LocalOnlyBehaviorTreeNodeState getState()
   {
      return state;
   }

   @Override
   public BehaviorTreeNodeDefinition getDefinition()
   {
      return state.getDefinition();
   }
}
