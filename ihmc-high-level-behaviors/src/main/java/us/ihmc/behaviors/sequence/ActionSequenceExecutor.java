package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;

public class ActionSequenceExecutor extends BehaviorTreeNodeExecutor<ActionSequenceState, ActionSequenceDefinition>
{
   private final ActionSequenceState state;

   public ActionSequenceExecutor(long id, CRDTInfo crdtInfo)
   {
      state = new ActionSequenceState(id, crdtInfo);
   }

   @Override
   public void tick()
   {
      super.tick();

      // TODO: Tick children


   }

   @Override
   public void update()
   {
      super.update();

      // TODO: Go through next for execution concurrent children and set chest action's goal pelvis frames
      //   and the hand pose actions chest frames

      for (BehaviorTreeNodeExecutor<?, ?> child : getChildren())
      {

      }
   }

   @Override
   public ActionSequenceState getState()
   {
      return state;
   }
}
