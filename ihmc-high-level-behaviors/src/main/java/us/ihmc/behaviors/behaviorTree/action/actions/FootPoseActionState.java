package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.FootPoseActionStateMessage;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
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
                                                   definition.getCRDTParentFrameName(),
                                                   definition.getFootToParentTransform());
   }

   @Override
   public void update()
   {
      footFrame.update();
   }

   public void toMessage(FootPoseActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(FootPoseActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }

   public CRDTDetachableReferenceFrame getFootFrame()
   {
      return footFrame;
   }
}
