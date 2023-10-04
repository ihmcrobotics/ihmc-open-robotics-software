package us.ihmc.behaviors.sequence.actions;

import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class FootstepPlanActionFootstepState
{
   private final FootstepPlanActionState footstepPlan;
   private final FootstepPlanActionFootstepDefinition definition = new FootstepPlanActionFootstepDefinition();
   private final DetachableReferenceFrame soleFrame;
   private int index = -1;

   public FootstepPlanActionFootstepState(ReferenceFrameLibrary referenceFrameLibrary, FootstepPlanActionState footstepPlan)
   {
      this.footstepPlan = footstepPlan;

      soleFrame = new DetachableReferenceFrame(referenceFrameLibrary, definition.getSoleToPlanFrameTransform());
   }

   public void update()
   {
      soleFrame.update(footstepPlan.getDefinition().getParentFrameName());
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
