package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class WalkActionState extends ActionNodeState<WalkActionDefinition>
{
   private final CRDTDetachableReferenceFrame goalFrame;
   private final FootstepPlanActionStateBasics footstepPlanStateBasics;
   private final CRDTUnidirectionalEnumField<WalkActionExecutionState> executionState;

   public WalkActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new WalkActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      goalFrame = new CRDTDetachableReferenceFrame(referenceFrameLibrary,
                                                   getDefinition().getBasics().getCRDTParentFrameName(),
                                                   getDefinition().getGoalToParentTransform());
      executionState = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.ROBOT, crdtInfo, WalkActionExecutionState.PLAN_EXECUTION_COMPLETE);
      footstepPlanStateBasics = new FootstepPlanActionStateBasics(crdtInfo);
   }

   @Override
   public void update()
   {
      goalFrame.update();
   }

   public void toMessage(WalkActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setExecutionState(executionState.toMessage().toByte());
      footstepPlanStateBasics.toMessage(message.getFootstepPlanStateBasics());
   }

   public void fromMessage(WalkActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      executionState.fromMessage(WalkActionExecutionState.fromByte(message.getExecutionState()));
      footstepPlanStateBasics.fromMessage(message.getFootstepPlanStateBasics());
   }

   public CRDTDetachableReferenceFrame getGoalFrame()
   {
      return goalFrame;
   }

   public CRDTUnidirectionalEnumField<WalkActionExecutionState> getExecutionState()
   {
      return executionState;
   }

   public FootstepPlanActionStateBasics getBasics()
   {
      return footstepPlanStateBasics;
   }
}
