package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;

public class ActionSequenceExecutor extends BehaviorTreeNodeExecutor<ActionSequenceState, ActionSequenceDefinition>
{
   private final ActionSequenceState state;

   public ActionSequenceExecutor(long id)
   {
      state = new ActionSequenceState(id);
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
