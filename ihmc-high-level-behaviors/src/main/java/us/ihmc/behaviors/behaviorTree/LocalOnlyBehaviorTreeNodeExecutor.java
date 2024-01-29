package us.ihmc.behaviors.behaviorTree;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

import java.util.ArrayList;
import java.util.List;

/**
 * This is currently around to keep older behavior tree nodes
 * compiling without them fully adhering to the newer standards.
 */
public abstract class LocalOnlyBehaviorTreeNodeExecutor extends BehaviorTreeNodeExecutor<LocalOnlyBehaviorTreeNodeState, BehaviorTreeNodeDefinition>
{
   // TODO: Fix
   private final List<LocalOnlyBehaviorTreeNodeExecutor> children = new ArrayList<>();

   public LocalOnlyBehaviorTreeNodeExecutor()
   {
      super(new LocalOnlyBehaviorTreeNodeState(new CRDTInfo(ROS2ActorDesignation.ROBOT, 5)));
   }

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

   // TODO: Fix
   public List<LocalOnlyBehaviorTreeNodeExecutor> getLocalOnlyChildren()
   {
      return children;
   }
}
