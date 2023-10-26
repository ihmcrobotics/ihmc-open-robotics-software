package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class WalkActionState extends ActionNodeState<WalkActionDefinition>
{
   private final DetachableReferenceFrame goalFrame;
   private int totalNumberOfFootsteps;
   private int numberOfIncompleteFootsteps;

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
      super.toMessage(message.getState());

      message.setTotalNumberOfFootsteps(totalNumberOfFootsteps);
      message.setNumberOfIncompleteFootsteps(numberOfIncompleteFootsteps);
   }

   public void fromMessage(WalkActionStateMessage message)
   {
      super.fromMessage(message.getState());

      totalNumberOfFootsteps = message.getTotalNumberOfFootsteps();
      numberOfIncompleteFootsteps = message.getNumberOfIncompleteFootsteps();
   }

   public DetachableReferenceFrame getGoalFrame()
   {
      return goalFrame;
   }

   public int getTotalNumberOfFootsteps()
   {
      return totalNumberOfFootsteps;
   }

   public void setTotalNumberOfFootsteps(int totalNumberOfFootsteps)
   {
      this.totalNumberOfFootsteps = totalNumberOfFootsteps;
   }

   public int getNumberOfIncompleteFootsteps()
   {
      return numberOfIncompleteFootsteps;
   }

   public void setNumberOfIncompleteFootsteps(int numberOfIncompleteFootsteps)
   {
      this.numberOfIncompleteFootsteps = numberOfIncompleteFootsteps;
   }
}
