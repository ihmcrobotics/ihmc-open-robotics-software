package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class PelvisHeightPitchActionState extends BehaviorActionState<PelvisHeightPitchActionDefinition>
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
      super.toMessage(message.getActionState());
   }

   public void fromMessage(PelvisHeightPitchActionStateMessage message)
   {
      super.fromMessage(message.getActionState());
   }

   public DetachableReferenceFrame getPelvisFrame()
   {
      return pelvisFrame;
   }
}
