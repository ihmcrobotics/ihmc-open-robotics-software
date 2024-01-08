package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalInteger;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class FootstepPlanActionStateBasics
{
   private final CRDTUnidirectionalInteger totalNumberOfFootsteps;
   private final CRDTUnidirectionalInteger numberOfIncompleteFootsteps;

   public FootstepPlanActionStateBasics(CRDTInfo crdtInfo)
   {
      totalNumberOfFootsteps = new CRDTUnidirectionalInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 0);
      numberOfIncompleteFootsteps = new CRDTUnidirectionalInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 0);
   }

   public void toMessage(FootstepPlanActionStateBasicsMessage message)
   {
      message.setTotalNumberOfFootsteps(totalNumberOfFootsteps.toMessage());
      message.setNumberOfIncompleteFootsteps(numberOfIncompleteFootsteps.toMessage());
   }

   public void fromMessage(FootstepPlanActionStateBasicsMessage message)
   {
      totalNumberOfFootsteps.fromMessage(message.getTotalNumberOfFootsteps());
      numberOfIncompleteFootsteps.fromMessage(message.getNumberOfIncompleteFootsteps());
   }

   public int getTotalNumberOfFootsteps()
   {
      return totalNumberOfFootsteps.getValue();
   }

   public void setTotalNumberOfFootsteps(int totalNumberOfFootsteps)
   {
      this.totalNumberOfFootsteps.setValue(totalNumberOfFootsteps);
   }

   public int getNumberOfIncompleteFootsteps()
   {
      return numberOfIncompleteFootsteps.getValue();
   }

   public void setNumberOfIncompleteFootsteps(int numberOfIncompleteFootsteps)
   {
      this.numberOfIncompleteFootsteps.setValue(numberOfIncompleteFootsteps);
   }
}
