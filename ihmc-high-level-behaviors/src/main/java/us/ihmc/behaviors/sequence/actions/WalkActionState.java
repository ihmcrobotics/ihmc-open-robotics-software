package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionDefinitionMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class WalkActionState extends BehaviorActionState<WalkActionDefinitionMessage>
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
