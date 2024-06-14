package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class PelvisHeightPitchActionState extends ActionNodeState<PelvisHeightOrientationActionDefinition>
{
   private final CRDTDetachableReferenceFrame pelvisFrame;

   public PelvisHeightPitchActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new PelvisHeightOrientationActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      pelvisFrame = new CRDTDetachableReferenceFrame(referenceFrameLibrary,
                                                     getDefinition().getCRDTParentFrameName(),
                                                     getDefinition().getPelvisToParentTransform());
   }

   @Override
   public void update()
   {
      pelvisFrame.update();
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

   public CRDTDetachableReferenceFrame getPelvisFrame()
   {
      return pelvisFrame;
   }
}
