package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalInteger;
import us.ihmc.communication.crdt.CRDTUnidirectionalPose3D;
import us.ihmc.communication.crdt.CRDTUnidirectionalSE3Trajectory;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepPlanActionStateBasics
{
   private final CRDTUnidirectionalInteger totalNumberOfFootsteps;
   private final CRDTUnidirectionalInteger numberOfIncompleteFootsteps;
   private final SideDependentList<CRDTUnidirectionalSE3Trajectory> desiredFootPoses = new SideDependentList<>();
   private final SideDependentList<CRDTUnidirectionalPose3D> currentFootPoses = new SideDependentList<>();

   public FootstepPlanActionStateBasics(CRDTInfo crdtInfo)
   {
      totalNumberOfFootsteps = new CRDTUnidirectionalInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 0);
      numberOfIncompleteFootsteps = new CRDTUnidirectionalInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 0);
      for (RobotSide side : RobotSide.values)
      {
         desiredFootPoses.set(side, new CRDTUnidirectionalSE3Trajectory(ROS2ActorDesignation.ROBOT, crdtInfo));
         currentFootPoses.set(side, new CRDTUnidirectionalPose3D(ROS2ActorDesignation.ROBOT, crdtInfo));
      }
   }

   public void toMessage(FootstepPlanActionStateBasicsMessage message)
   {
      message.setTotalNumberOfFootsteps(totalNumberOfFootsteps.toMessage());
      message.setNumberOfIncompleteFootsteps(numberOfIncompleteFootsteps.toMessage());
      desiredFootPoses.get(RobotSide.LEFT).toMessage(message.getDesiredLeftFootsteps());
      desiredFootPoses.get(RobotSide.RIGHT).toMessage(message.getDesiredRightFootsteps());
      currentFootPoses.get(RobotSide.LEFT).toMessage(message.getCurrentLeftFootPose());
      currentFootPoses.get(RobotSide.RIGHT).toMessage(message.getCurrentRightFootPose());
   }

   public void fromMessage(FootstepPlanActionStateBasicsMessage message)
   {
      totalNumberOfFootsteps.fromMessage(message.getTotalNumberOfFootsteps());
      numberOfIncompleteFootsteps.fromMessage(message.getNumberOfIncompleteFootsteps());
      desiredFootPoses.get(RobotSide.LEFT).fromMessage(message.getDesiredLeftFootsteps());
      desiredFootPoses.get(RobotSide.RIGHT).fromMessage(message.getDesiredRightFootsteps());
      currentFootPoses.get(RobotSide.LEFT).fromMessage(message.getCurrentLeftFootPose());
      currentFootPoses.get(RobotSide.RIGHT).fromMessage(message.getCurrentRightFootPose());
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

   public SideDependentList<CRDTUnidirectionalSE3Trajectory> getDesiredFootPoses()
   {
      return desiredFootPoses;
   }

   public SideDependentList<CRDTUnidirectionalPose3D> getCurrentFootPoses()
   {
      return currentFootPoses;
   }
}