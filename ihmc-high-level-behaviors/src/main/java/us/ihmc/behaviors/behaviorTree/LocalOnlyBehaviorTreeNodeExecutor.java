package us.ihmc.behaviors.behaviorTree;

import java.util.ArrayList;
import java.util.List;

/**
 * This is currently around to keep older behavior tree nodes
 * compiling without them fully adhering to the newer standards.
 */
public abstract class LocalOnlyBehaviorTreeNodeExecutor extends BehaviorTreeNodeExecutor
{
   private final LocalOnlyBehaviorTreeNodeState state = new LocalOnlyBehaviorTreeNodeState();

   // TODO: Fix
   private final List<LocalOnlyBehaviorTreeNodeExecutor> children = new ArrayList<>();

   @Override
   public void tick()
   {
      super.tick();

      getState().setStatus(determineStatus());
   }

   public BehaviorTreeNodeStatus tickAndGetStatus()
   {
      tick();
      return getState().getStatus();
   }

   public abstract BehaviorTreeNodeStatus determineStatus();

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

   // TODO: Fix
   public List<LocalOnlyBehaviorTreeNodeExecutor> getLocalOnlyChildren()
   {
      return children;
   }
}
