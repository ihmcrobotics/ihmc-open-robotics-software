package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class PelvisHeightPitchActionState extends ActionNodeState<PelvisHeightPitchActionDefinition>
{
   private final DetachableReferenceFrame pelvisFrame;

   public PelvisHeightPitchActionState(long id, ROS2ActorDesignation actorDesignation, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new PelvisHeightPitchActionDefinition(), actorDesignation);

      pelvisFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getPelvisToParentTransform());
   }

   @Override
   public void update()
   {
      pelvisFrame.update(getDefinition().getParentFrameName());
      setCanExecute(pelvisFrame.isChildOfWorld());
   }

   public void toMessage(PelvisHeightPitchActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(PelvisHeightPitchActionStateMessage message)
   {
      getDefinition().fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }

   public DetachableReferenceFrame getPelvisFrame()
   {
      return pelvisFrame;
   }
}
