package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SidedBodyPartPoseActionDefinitionMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class HandPoseActionState extends BehaviorActionState<SidedBodyPartPoseActionDefinitionMessage>
{
   private final HandPoseActionDefinition definition = new HandPoseActionDefinition();
   private final DetachableReferenceFrame palmFrame;

   public HandPoseActionState(ReferenceFrameLibrary referenceFrameLibrary)
   {
      palmFrame = new DetachableReferenceFrame(referenceFrameLibrary, definition.getPalmTransformToParent());
   }

   @Override
   public void update()
   {
      palmFrame.update(definition.getPalmParentFrameName());
   }

   @Override
   public HandPoseActionDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getPalmFrame()
   {
      return palmFrame;
   }
}
