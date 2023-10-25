package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class HandPoseActionState extends ActionNodeState<HandPoseActionDefinition>
{
   private final DetachableReferenceFrame palmFrame;
   private double handWrenchMagnitudeLinear;

   public HandPoseActionState(long id, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new HandPoseActionDefinition());

      palmFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getPalmTransformToParent());
   }

   @Override
   public void update()
   {
      palmFrame.update(getDefinition().getPalmParentFrameName());
      setCanExecute(palmFrame.isChildOfWorld());
   }

   public void toMessage(HandPoseActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      message.setHandWrenchMagnitudeLinear(handWrenchMagnitudeLinear);
   }

   public void fromMessage(HandPoseActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      handWrenchMagnitudeLinear = message.getHandWrenchMagnitudeLinear();
   }

   public DetachableReferenceFrame getPalmFrame()
   {
      return palmFrame;
   }

   public double getHandWrenchMagnitudeLinear()
   {
      return handWrenchMagnitudeLinear;
   }

   public void setHandWrenchMagnitudeLinear(double handWrenchMagnitudeLinear)
   {
      this.handWrenchMagnitudeLinear = handWrenchMagnitudeLinear;
   }
}
