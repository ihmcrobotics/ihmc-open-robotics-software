package us.ihmc.behaviors.behaviorTree;

import java.time.Instant;

/**
 * This is currently around to keep older behavior tree nodes
 * compiling without them fully adhering to the newer standards.
 */
public class LocalOnlyBehaviorTreeNodeExecutor extends BehaviorTreeNodeExecutor
{
   private final LocalOnlyBehaviorTreeNodeState state = new LocalOnlyBehaviorTreeNodeState();


   public BehaviorTreeNodeStatus tick()
   {
      state.setStatus(tickInternal());
      state.setLastTickInstant(Instant.now());
      return state.getStatus();
   }

   // TODO: Is this a good convention doing the *Internal thing?
   //   or would it be better to call super.tick or something somehow
   public BehaviorTreeNodeStatus tickInternal()
   {
      return state.getStatus();
   }


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
