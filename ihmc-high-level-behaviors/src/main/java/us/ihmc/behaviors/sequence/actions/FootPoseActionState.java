package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootPoseActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class FootPoseActionState extends ActionNodeState<FootPoseActionDefinition>
{
   private final CRDTDetachableReferenceFrame footFrame;

   public FootPoseActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new FootPoseActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      footFrame = new CRDTDetachableReferenceFrame(referenceFrameLibrary,
                                                    getDefinition().getCRDTParentFrameName(),
                                                    getDefinition().getFootToParentTransform());
   }

   @Override
   public void update()
   {
      footFrame.update();
   }

   public void toMessage(FootPoseActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(FootPoseActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }

   public CRDTDetachableReferenceFrame getFootFrame()
   {
      return footFrame;
   }
}
