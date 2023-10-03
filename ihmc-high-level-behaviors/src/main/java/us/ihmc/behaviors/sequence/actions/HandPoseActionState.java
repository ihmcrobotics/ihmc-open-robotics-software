package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class HandPoseActionState extends BehaviorActionState
{
   private final HandPoseActionDefinition definition = new HandPoseActionDefinition();
   private final DetachableReferenceFrame palmFrame = new DetachableReferenceFrame(definition.getPalmTransformToParent());

   @Override
   public HandPoseActionDefinition getDefinition()
   {
      return definition;
   }

   @Override
   public void update(ReferenceFrameLibrary referenceFrameLibrary)
   {
      palmFrame.update(referenceFrameLibrary, definition.getPalmParentFrameName());
   }

}
