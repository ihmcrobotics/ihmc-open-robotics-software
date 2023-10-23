package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ChestOrientationActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class ChestOrientationActionState extends BehaviorActionState
{
   private final ChestOrientationActionDefinition definition;
   private final DetachableReferenceFrame chestFrame;

   public ChestOrientationActionState(long id, ChestOrientationActionDefinition definition, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, definition);

      this.definition = definition;

      chestFrame = new DetachableReferenceFrame(referenceFrameLibrary, definition.getChestToParentTransform());
   }

   @Override
   public void update()
   {
      chestFrame.update(definition.getParentFrameName());
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

   @Override
   public ChestOrientationActionDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getChestFrame()
   {
      return chestFrame;
   }
}
