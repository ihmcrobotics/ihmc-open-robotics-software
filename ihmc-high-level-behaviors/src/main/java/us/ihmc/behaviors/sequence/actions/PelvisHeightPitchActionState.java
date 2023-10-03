package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.BodyPartPoseActionDefinitionMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class PelvisHeightPitchActionState extends BehaviorActionState<BodyPartPoseActionDefinitionMessage>
{
   private final PelvisHeightPitchActionDefinition definition = new PelvisHeightPitchActionDefinition();
   private final DetachableReferenceFrame pelvisFrame = new DetachableReferenceFrame(definition.getTransformToParent());

   @Override
   public void update(ReferenceFrameLibrary referenceFrameLibrary)
   {
      pelvisFrame.update(referenceFrameLibrary, definition.getParentFrameName());
   }

   @Override
   public PelvisHeightPitchActionDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getPelvisFrame()
   {
      return pelvisFrame;
   }
}
