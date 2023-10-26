package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class PelvisHeightPitchActionState extends ActionNodeState<PelvisHeightPitchActionDefinition>
{
   private final DetachableReferenceFrame pelvisFrame;

   public PelvisHeightPitchActionState(long id, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new PelvisHeightPitchActionDefinition());

      pelvisFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getPelvisToParentTransform());
   }

   @Override
   public void update()
   {
      pelvisFrame.update(getDefinition().getParentFrameName());
      setCanExecute(pelvisFrame.isChildOfWorld());
   }

   public void toMessage(PelvisHeightPitchActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(PelvisHeightPitchActionStateMessage message)
   {
      getDefinition().fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }

   public DetachableReferenceFrame getPelvisFrame()
   {
      return pelvisFrame;
   }
}
