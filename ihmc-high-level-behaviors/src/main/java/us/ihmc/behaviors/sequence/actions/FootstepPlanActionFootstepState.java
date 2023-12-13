package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class FootstepPlanActionFootstepState
{
   private final FootstepPlanActionState footstepPlan;
   private final FootstepPlanActionFootstepDefinition definition;
   private final CRDTDetachableReferenceFrame soleFrame;
   private int index = -1;

   public FootstepPlanActionFootstepState(ReferenceFrameLibrary referenceFrameLibrary,
                                          FootstepPlanActionState footstepPlan,
                                          FootstepPlanActionFootstepDefinition definition)
   {
      this.footstepPlan = footstepPlan;
      this.definition = definition;

      soleFrame = new CRDTDetachableReferenceFrame(referenceFrameLibrary,
                                                   footstepPlan.getDefinition().getCRDTParentFrameName(),
                                                   definition.getSoleToPlanFrameTransform());
   }

   public void update()
   {
      soleFrame.update();
   }

   public void toMessage(FootstepPlanActionFootstepStateMessage message)
   {
      message.setIndex(index);
   }

   public void fromMessage(FootstepPlanActionFootstepStateMessage message)
   {
      index = message.getIndex();
   }

   public FootstepPlanActionFootstepDefinition getDefinition()
   {
      return definition;
   }

   public CRDTDetachableReferenceFrame getSoleFrame()
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
