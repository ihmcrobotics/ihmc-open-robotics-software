package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ChestOrientationActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalRigidBodyTransform;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class ChestOrientationActionState extends ActionNodeState<ChestOrientationActionDefinition>
{
   private final DetachableReferenceFrame chestFrame;
   /**
    * This is the estimated goal pelvis frame as the robot executes a potential whole body action.
    * This is used to compute joint angles that achieve the desired and previewed end pose
    * even when the pelvis and/or chest might also move.
    */
   private final CRDTUnidirectionalRigidBodyTransform goalPelvisToWorldTransform;
   private final ReferenceFrame goalPelvisFrame;

   public ChestOrientationActionState(long id, CRDTInfo crdtInfo, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new ChestOrientationActionDefinition(crdtInfo), crdtInfo);

      chestFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getChestToParentTransform().getValueReadOnly());
      goalPelvisToWorldTransform = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.ROBOT, crdtInfo);
      goalPelvisFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                               goalPelvisToWorldTransform.getValueReadOnly());
   }

   @Override
   public void update()
   {
      chestFrame.update(getDefinition().getParentFrameName());
   }

   public void toMessage(ChestOrientationActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      goalPelvisToWorldTransform.toMessage(message.getGoalPelvisTransformToWorld());
   }

   public void fromMessage(ChestOrientationActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      goalPelvisToWorldTransform.fromMessage(message.getGoalPelvisTransformToWorld());
      goalPelvisFrame.update();
   }

   public DetachableReferenceFrame getChestFrame()
   {
      return chestFrame;
   }

   public CRDTUnidirectionalRigidBodyTransform getGoalPelvisToWorldTransform()
   {
      return goalPelvisToWorldTransform;
   }

   public ReferenceFrame getGoalPelvisFrame()
   {
      return goalPelvisFrame;
   }
}
