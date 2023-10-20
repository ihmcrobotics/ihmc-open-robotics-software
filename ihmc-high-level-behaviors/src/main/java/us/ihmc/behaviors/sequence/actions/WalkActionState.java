package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class WalkActionState extends BehaviorActionState
{
   private final WalkActionDefinition definition;
   private final DetachableReferenceFrame goalFrame;

   public WalkActionState(ReferenceFrameLibrary referenceFrameLibrary, FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      definition = new WalkActionDefinition(footstepPlannerParameters);
      goalFrame = new DetachableReferenceFrame(referenceFrameLibrary, definition.getGoalToParentTransform());
   }

   @Override
   public void update()
   {
      goalFrame.update(definition.getParentFrameName());
      setCanExecute(goalFrame.isChildOfWorld());
   }

   public void toMessage(WalkActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      definition.toMessage(message.getDefinition());
   }

   public void fromMessage(WalkActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      definition.fromMessage(message.getDefinition());
   }

   @Override
   public WalkActionDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getGoalFrame()
   {
      return goalFrame;
   }
}
