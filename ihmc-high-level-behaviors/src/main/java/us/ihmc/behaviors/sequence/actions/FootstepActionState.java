package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepActionDefinitionMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class FootstepActionState extends BehaviorActionState<FootstepActionDefinitionMessage>
{
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final FootstepPlanActionState footstepPlan;
   private final FootstepActionDefinition definition = new FootstepActionDefinition();
   private final DetachableReferenceFrame soleFrame;

   public FootstepActionState(ReferenceFrameLibrary referenceFrameLibrary, FootstepPlanActionState footstepPlan)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.footstepPlan = footstepPlan;

      soleFrame = new DetachableReferenceFrame(referenceFrameLibrary, definition.getSoleToPlanFrameTransform());
   }

   @Override
   public void update()
   {
      soleFrame.update(footstepPlan.getDefinition().getParentFrameName());
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
