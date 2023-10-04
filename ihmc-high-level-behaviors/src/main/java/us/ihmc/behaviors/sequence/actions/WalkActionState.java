package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionDefinitionMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class WalkActionState extends BehaviorActionState<WalkActionDefinitionMessage>
{
   private final WalkActionDefinition definition = new WalkActionDefinition();
   private final DetachableReferenceFrame soleFrame;

   public WalkActionState(ReferenceFrameLibrary referenceFrameLibrary)
   {
      soleFrame = new DetachableReferenceFrame(referenceFrameLibrary, definition.getTransformToParent());
   }

   @Override
   public void update()
   {
      soleFrame.update(definition.getParentFrameName());
   }

   @Override
   public WalkActionDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getSoleFrame()
   {
      return soleFrame;
   }
}
