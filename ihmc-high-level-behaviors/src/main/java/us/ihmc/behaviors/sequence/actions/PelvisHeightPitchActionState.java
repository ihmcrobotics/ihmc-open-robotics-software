package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.BodyPartPoseActionDefinitionMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;

public class PelvisHeightPitchActionState extends BehaviorActionState<BodyPartPoseActionDefinitionMessage>
{
   private final PelvisHeightPitchActionDefinition definition = new PelvisHeightPitchActionDefinition();
   private final DetachableReferenceFrame pelvisFrame = new DetachableReferenceFrame(definition.getTransformToParent());

   @Override
   public void update()
   {
      pelvisFrame.update(definition.getParentFrameName());
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
