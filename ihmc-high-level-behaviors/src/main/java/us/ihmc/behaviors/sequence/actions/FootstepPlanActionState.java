package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage;
import behavior_msgs.msg.dds.FootstepPlanActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.lists.RecyclingArrayListTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class FootstepPlanActionState extends BehaviorActionState
{
   private final FootstepPlanActionDefinition definition;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private int numberOfAllocatedFootsteps = 0;
   private final RecyclingArrayList<FootstepPlanActionFootstepState> footsteps;

   public FootstepPlanActionState(long id, FootstepPlanActionDefinition definition, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, definition);

      this.definition = definition;
      this.referenceFrameLibrary = referenceFrameLibrary;

      footsteps = new RecyclingArrayList<>(() ->
         new FootstepPlanActionFootstepState(referenceFrameLibrary,
                                             this,
                                             RecyclingArrayListTools.getUnsafe(definition.getFootsteps(), numberOfAllocatedFootsteps++)));
   }

   @Override
   public void update()
   {
      RecyclingArrayListTools.synchronizeSize(footsteps, definition.getFootsteps());

      for (int i = 0; i < footsteps.size(); i++)
      {
         footsteps.get(i).setIndex(i);
         footsteps.get(i).update();
      }

      setCanExecute(referenceFrameLibrary.containsFrame(definition.getParentFrameName()));
   }

   public void toMessage(FootstepPlanActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      message.getFootsteps().clear();
      for (FootstepPlanActionFootstepState footstep : footsteps)
      {
         footstep.toMessage(message.getFootsteps().add());
      }
   }

   public void fromMessage(FootstepPlanActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      footsteps.clear();
      for (FootstepPlanActionFootstepStateMessage footstep : message.getFootsteps())
      {
         footsteps.add().fromMessage(footstep);
      }
   }

   public RecyclingArrayList<FootstepPlanActionFootstepState> getFootsteps()
   {
      return footsteps;
   }

   @Override
   public FootstepPlanActionDefinition getDefinition()
   {
      return definition;
   }
}
