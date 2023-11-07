package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ChestOrientationActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class ChestOrientationActionState extends ActionNodeState<ChestOrientationActionDefinition>
{
   private final DetachableReferenceFrame chestFrame;
   /**
    * This is the estimated goal pelvis frame as the robot executes a potential whole body action.
    * This is used to compute joint angles that achieve the desired and previewed end pose
    * even when the pelvis and/or chest might also move.
    */
   private final MutableReferenceFrame goalPelvisFrame = new MutableReferenceFrame();

   public ChestOrientationActionState(long id, CRDTInfo crdtInfo, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new ChestOrientationActionDefinition(crdtInfo), crdtInfo);

      chestFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getChestToParentTransform().getValueReadOnly());
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

      MessageTools.toMessage(goalPelvisFrame.getTransformToParent(), message.getGoalPelvisTransformToWorld());
   }

   public void fromMessage(ChestOrientationActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      MessageTools.toEuclid(message.getGoalPelvisTransformToWorld(), goalPelvisFrame.getTransformToParent());
      goalPelvisFrame.getReferenceFrame().update();
   }

   public DetachableReferenceFrame getChestFrame()
   {
      return chestFrame;
   }

   public MutableReferenceFrame getGoalPelvisFrame()
   {
      return goalPelvisFrame;
   }
}
