package us.ihmc.behaviors.behaviorTree;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;

public abstract class BehaviorTreeNodeExecutor implements BehaviorTreeNodeStateSupplier
{
   private final List<BehaviorTreeNodeExecutor> children = new ArrayList<>();

   public void update()
   {
      getState().clock();
   }

   public BehaviorTreeNodeStatus tick()
   {
      getState().setStatus(tickInternal());
      getState().setLastTickInstant(Instant.now());
      return getState().getStatus();
   }

   // TODO: Is this a good convention doing the *Internal thing?
   //   or would it be better to call super.tick or something somehow
   protected BehaviorTreeNodeStatus tickInternal()
   {
      return getState().getStatus();
   }

   public List<BehaviorTreeNodeExecutor> getChildren()
   {
      return children;
   }
}
