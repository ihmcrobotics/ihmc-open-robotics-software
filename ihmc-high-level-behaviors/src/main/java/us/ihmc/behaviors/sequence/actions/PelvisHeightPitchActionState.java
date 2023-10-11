package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class PelvisHeightPitchActionState extends BehaviorActionState
{
   private final PelvisHeightPitchActionDefinition definition = new PelvisHeightPitchActionDefinition();
   private final DetachableReferenceFrame pelvisFrame;

   public PelvisHeightPitchActionState(ReferenceFrameLibrary referenceFrameLibrary)
   {
      pelvisFrame = new DetachableReferenceFrame(referenceFrameLibrary, definition.getPelvisToParentTransform());
   }

   @Override
   public void update()
   {
      pelvisFrame.update(definition.getParentFrameName());
      setCanExecute(pelvisFrame.isChildOfWorld());
   }

   public void toMessage(PelvisHeightPitchActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      definition.toMessage(message.getDefinition());
   }

   public void fromMessage(PelvisHeightPitchActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      definition.fromMessage(message.getDefinition());
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
