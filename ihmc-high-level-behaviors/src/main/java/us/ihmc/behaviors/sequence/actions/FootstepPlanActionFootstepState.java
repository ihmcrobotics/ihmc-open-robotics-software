package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class FootstepPlanActionFootstepState
{
   private final FootstepPlanActionState footstepPlan;
   private final FootstepPlanActionFootstepDefinition definition;
   private final DetachableReferenceFrame soleFrame;
   private int index = -1;

   public FootstepPlanActionFootstepState(ReferenceFrameLibrary referenceFrameLibrary,
                                          FootstepPlanActionState footstepPlan,
                                          FootstepPlanActionFootstepDefinition definition)
   {
      this.footstepPlan = footstepPlan;
      this.definition = definition;

      soleFrame = new DetachableReferenceFrame(referenceFrameLibrary, definition.getSoleToPlanFrameTransform());
   }

   public void update()
   {
      soleFrame.update(footstepPlan.getDefinition().getParentFrameName());
   }

   public void toMessage(FootstepPlanActionFootstepStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      message.setIndex(index);
   }

   public void fromMessage(FootstepPlanActionFootstepStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      index = message.getIndex();
   }

   public FootstepPlanActionFootstepDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getSoleFrame()
   {
      return soleFrame;
   }

   public int getIndex()
   {
      return index;
   }

   public void setIndex(int index)
   {
      this.index = index;
   }
}
