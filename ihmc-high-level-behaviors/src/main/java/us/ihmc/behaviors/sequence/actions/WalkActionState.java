package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class WalkActionState extends BehaviorActionState<WalkActionDefinition>
{
   private final DetachableReferenceFrame goalFrame;

   public WalkActionState(long id, FootstepPlannerParametersBasics footstepPlannerParameters, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new WalkActionDefinition(footstepPlannerParameters));

      goalFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getGoalToParentTransform());
   }

   @Override
   public void update()
   {
      goalFrame.update(getDefinition().getParentFrameName());
      setCanExecute(goalFrame.isChildOfWorld());
   }

   public void toMessage(WalkActionStateMessage message)
   {
      super.toMessage(message.getActionState());
   }

   public void fromMessage(WalkActionStateMessage message)
   {
      super.fromMessage(message.getActionState());
   }

   public DetachableReferenceFrame getGoalFrame()
   {
      return goalFrame;
   }
}
