package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.KickDoorActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalEnumField;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class KickDoorActionState extends ActionNodeState<KickDoorActionDefinition>
{
   private final ReferenceFrame parentFrame;
   private final KickDoorActionDefinition definition;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private CRDTUnidirectionalEnumField<KickDoorActionExecutionState> executionState;

   public KickDoorActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new KickDoorActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      definition = getDefinition();

      this.referenceFrameLibrary = referenceFrameLibrary;

      parentFrame = referenceFrameLibrary.findFrameByName(definition.getParentFrameName());

      executionState = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.ROBOT, crdtInfo, KickDoorActionExecutionState.STANDING);
   }

   @Override
   public void update()
   {
   }

   public void toMessage(KickDoorActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setExecutionState(executionState.toMessage().toByte());
   }

   public void fromMessage(KickDoorActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      executionState.fromMessage(KickDoorActionExecutionState.fromByte(message.getExecutionState()));
   }

   public boolean areFramesInWorld()
   {
      return referenceFrameLibrary.containsFrame(definition.getParentFrameName()) && parentFrame.getRootFrame() == ReferenceFrame.getWorldFrame();
   }

   public ReferenceFrame getParentFrame()
   {
      return parentFrame;
   }

   public CRDTUnidirectionalEnumField<KickDoorActionExecutionState> getExecutionState()
   {
      return executionState;
   }
}
