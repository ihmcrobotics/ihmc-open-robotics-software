package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalInteger;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class WalkActionState extends ActionNodeState<WalkActionDefinition>
{
   private final DetachableReferenceFrame goalFrame;
   private final CRDTUnidirectionalInteger totalNumberOfFootsteps;
   private final CRDTUnidirectionalInteger numberOfIncompleteFootsteps;

   public WalkActionState(long id,
                          CRDTInfo crdtInfo,
                          FootstepPlannerParametersBasics footstepPlannerParameters,
                          ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new WalkActionDefinition(footstepPlannerParameters), crdtInfo);

      goalFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getGoalToParentTransform());
      totalNumberOfFootsteps = new CRDTUnidirectionalInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 0);
      numberOfIncompleteFootsteps = new CRDTUnidirectionalInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 0);
   }

   @Override
   public void update()
   {
      goalFrame.update(getDefinition().getParentFrameName());
   }

   public void toMessage(WalkActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setTotalNumberOfFootsteps(totalNumberOfFootsteps.toMessage());
      message.setNumberOfIncompleteFootsteps(numberOfIncompleteFootsteps.toMessage());
   }

   public void fromMessage(WalkActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      totalNumberOfFootsteps.fromMessage(message.getTotalNumberOfFootsteps());
      numberOfIncompleteFootsteps.fromMessage(message.getNumberOfIncompleteFootsteps());
   }

   public DetachableReferenceFrame getGoalFrame()
   {
      return goalFrame;
   }

   public int getTotalNumberOfFootsteps()
   {
      return totalNumberOfFootsteps.intValue();
   }

   public void setTotalNumberOfFootsteps(int totalNumberOfFootsteps)
   {
      this.totalNumberOfFootsteps.setValue(totalNumberOfFootsteps);
   }

   public int getNumberOfIncompleteFootsteps()
   {
      return numberOfIncompleteFootsteps.intValue();
   }

   public void setNumberOfIncompleteFootsteps(int numberOfIncompleteFootsteps)
   {
      this.numberOfIncompleteFootsteps.setValue(numberOfIncompleteFootsteps);
   }
}
