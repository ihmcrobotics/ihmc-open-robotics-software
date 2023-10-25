package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ChestOrientationActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class ChestOrientationActionState extends ActionNodeState<ChestOrientationActionDefinition>
{
   private final DetachableReferenceFrame chestFrame;

   public ChestOrientationActionState(long id, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new ChestOrientationActionDefinition());

      chestFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getChestToParentTransform());
   }

   @Override
   public void update()
   {
      chestFrame.update(getDefinition().getParentFrameName());
      setCanExecute(chestFrame.isChildOfWorld());
   }

   public void toMessage(ChestOrientationActionStateMessage message)
   {
      super.toMessage(message.getActionState());
   }

   public void fromMessage(ChestOrientationActionStateMessage message)
   {
      super.fromMessage(message.getActionState());
   }

   public DetachableReferenceFrame getChestFrame()
   {
      return chestFrame;
   }
}
