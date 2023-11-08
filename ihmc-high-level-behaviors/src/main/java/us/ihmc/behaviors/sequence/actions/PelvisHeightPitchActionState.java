package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class PelvisHeightPitchActionState extends ActionNodeState<PelvisHeightPitchActionDefinition>
{
   private final DetachableReferenceFrame pelvisFrame;

   public PelvisHeightPitchActionState(long id, CRDTInfo crdtInfo, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new PelvisHeightPitchActionDefinition(crdtInfo), crdtInfo);

      pelvisFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getPelvisToParentTransform().getValueReadOnly());
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
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }

   public DetachableReferenceFrame getPelvisFrame()
   {
      return pelvisFrame;
   }
}
