package us.ihmc.behaviors.behaviorTree;

/**
 * This is currently around to keep older behavior tree nodes
 * compiling without them fully adhering to the newer standards.
 */
public class LegacyBehaviorTreeNodeState extends BehaviorTreeNodeState
{
   private final BehaviorTreeNodeDefinition definition = new BehaviorTreeNodeDefinition();

   @Override
   public BehaviorTreeNodeDefinition getDefinition()
   {
      return definition;
   }
}
