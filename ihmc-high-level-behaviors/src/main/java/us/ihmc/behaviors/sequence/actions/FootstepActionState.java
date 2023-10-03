package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class FootstepActionState extends BehaviorActionState
{
   private final FootstepPlanActionState footstepPlan;
   private final FootstepActionDefinition definition = new FootstepActionDefinition();
   private final DetachableReferenceFrame soleFrame = new DetachableReferenceFrame(definition.getSoleToPlanFrameTransform());

   public FootstepActionState(FootstepPlanActionState footstepPlan)
   {
      this.footstepPlan = footstepPlan;
   }

   @Override
   public void update(ReferenceFrameLibrary referenceFrameLibrary)
   {
      soleFrame.update(referenceFrameLibrary, footstepPlan.getDefinition().getParentFrameName());
   }

   @Override
   public FootstepActionDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getSoleFrame()
   {
      return soleFrame;
   }
}
