package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class HandPoseActionState extends BehaviorActionState
{
   private final HandPoseActionDefinition definition;
   private final DetachableReferenceFrame palmFrame;

   public HandPoseActionState(long id, HandPoseActionDefinition definition, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, definition);

      this.definition = definition;

      palmFrame = new DetachableReferenceFrame(referenceFrameLibrary, definition.getPalmTransformToParent());
   }

   @Override
   public void update()
   {
      palmFrame.update(definition.getPalmParentFrameName());
      setCanExecute(palmFrame.isChildOfWorld());
   }

   public void toMessage(HandPoseActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      definition.toMessage(message.getDefinition());
   }

   public void fromMessage(HandPoseActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      definition.fromMessage(message.getDefinition());
   }

   @Override
   public HandPoseActionDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getPalmFrame()
   {
      return palmFrame;
   }
}
