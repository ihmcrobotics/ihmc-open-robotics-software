package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class ActionSequenceExecutor extends BehaviorTreeNodeExecutor<ActionSequenceState, ActionSequenceDefinition>
{
   private final ActionSequenceState state;

   public ActionSequenceExecutor(long id)
   {
      state = new ActionSequenceState(id, ROS2ActorDesignation.ROBOT);
   }

   @Override
   public void tick()
   {
      super.tick();

      // TODO: Tick children
   }

   public void update()
   {
      // TODO: Go through next for execution concurrent children and set chest action's goal pelvis frames
      //   and the hand pose actions chest frames
   }

   @Override
   public ActionSequenceState getState()
   {
      return state;
   }
}
