package us.ihmc.behaviors.behaviorTree.action;

import behavior_msgs.msg.dds.ActionNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.communication.crdt.CRDTStatusDoubleArray;
import us.ihmc.communication.crdt.CRDTStatusOneDoFJointTrajectoryList;
import us.ihmc.communication.crdt.CRDTStatusPose3D;
import us.ihmc.communication.crdt.CRDTStatusSE3Trajectory;
import us.ihmc.communication.crdt.CRDTStatusDouble;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public abstract class ActionNodeState<D extends ActionNodeDefinition> extends LeafNodeState<D>
{
   public static final int SUPPORTED_NUMBER_OF_JOINTS = 7;

   private final CRDTStatusDouble nominalExecutionDuration;
   private final CRDTStatusDouble elapsedExecutionTime;
   private final CRDTStatusSE3Trajectory commandedTrajectory;
   private final CRDTStatusPose3D currentPose;
   private final CRDTStatusOneDoFJointTrajectoryList commandedJointTrajectories;
   private final CRDTStatusDoubleArray currentJointAngles;
   private final CRDTStatusDouble positionDistanceToGoalTolerance;
   private final CRDTStatusDouble orientationDistanceToGoalTolerance;

   public ActionNodeState(long id, D definition, BehaviorTreeRootNodeState rootNode)
   {
      super(id, definition, rootNode);

      nominalExecutionDuration = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      elapsedExecutionTime = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      commandedTrajectory = new CRDTStatusSE3Trajectory(ROS2ActorDesignation.ROBOT, crdtInfo);
      currentPose = new CRDTStatusPose3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      commandedJointTrajectories = new CRDTStatusOneDoFJointTrajectoryList(ROS2ActorDesignation.ROBOT, crdtInfo);
      currentJointAngles = new CRDTStatusDoubleArray(ROS2ActorDesignation.ROBOT, crdtInfo, SUPPORTED_NUMBER_OF_JOINTS);
      positionDistanceToGoalTolerance = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      orientationDistanceToGoalTolerance = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
   }

   @Override
   public boolean hasStatus()
   {
      boolean hasStatus = super.hasStatus();
      hasStatus |= nominalExecutionDuration.pollHasStatus();
      hasStatus |= elapsedExecutionTime.pollHasStatus();
      hasStatus |= commandedTrajectory.pollHasStatus();
      hasStatus |= currentPose.pollHasStatus();
      hasStatus |= commandedJointTrajectories.pollHasStatus();
      hasStatus |= currentJointAngles.pollHasStatus();
      hasStatus |= positionDistanceToGoalTolerance.pollHasStatus();
      hasStatus |= orientationDistanceToGoalTolerance.pollHasStatus();
      return hasStatus;
   }

   public void toMessage(ActionNodeStateMessage message)
   {
      super.toMessage(message.getState());

      message.setNominalExecutionDuration(nominalExecutionDuration.toMessage());
      message.setElapsedExecutionTime(elapsedExecutionTime.toMessage());
      commandedTrajectory.toMessage(message.getCommandedTrajectory());
      currentPose.toMessage(message.getCurrentPose());
      commandedJointTrajectories.toMessage(message.getCommandedJointTrajectories());
      currentJointAngles.toMessage(message.getCurrentJointAngles());
      message.setPositionDistanceToGoalTolerance(positionDistanceToGoalTolerance.toMessage());
      message.setOrientationDistanceToGoalTolerance(orientationDistanceToGoalTolerance.toMessage());
   }

   public void fromMessage(ActionNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      nominalExecutionDuration.fromMessage(message.getNominalExecutionDuration());
      elapsedExecutionTime.fromMessage(message.getElapsedExecutionTime());
      commandedTrajectory.fromMessage(message.getCommandedTrajectory());
      currentPose.fromMessage(message.getCurrentPose());
      commandedJointTrajectories.fromMessage(message.getCommandedJointTrajectories());
      currentJointAngles.fromMessage(message.getCurrentJointAngles());
      positionDistanceToGoalTolerance.fromMessage(message.getPositionDistanceToGoalTolerance());
      orientationDistanceToGoalTolerance.fromMessage(message.getOrientationDistanceToGoalTolerance());
   }

   public void setNominalExecutionDuration(double nominalExecutionDuration)
   {
      this.nominalExecutionDuration.setValue(nominalExecutionDuration);
   }

   public double getNominalExecutionDuration()
   {
      return nominalExecutionDuration.getValue();
   }

   public void setElapsedExecutionTime(double elapsedExecutionTime)
   {
      this.elapsedExecutionTime.setValue(elapsedExecutionTime);
   }

   public double getElapsedExecutionTime()
   {
      return elapsedExecutionTime.getValue();
   }

   public CRDTStatusSE3Trajectory getCommandedTrajectory()
   {
      return commandedTrajectory;
   }

   public CRDTStatusPose3D getCurrentPose()
   {
      return currentPose;
   }

   public CRDTStatusOneDoFJointTrajectoryList getCommandedJointTrajectories()
   {
      return commandedJointTrajectories;
   }

   public CRDTStatusDoubleArray getCurrentJointAngles()
   {
      return currentJointAngles;
   }

   public double getPositionDistanceToGoalTolerance()
   {
      return positionDistanceToGoalTolerance.getValue();
   }

   public void setPositionDistanceToGoalTolerance(double positionDistanceToGoalTolerance)
   {
      this.positionDistanceToGoalTolerance.setValue(positionDistanceToGoalTolerance);
   }

   public double getOrientationDistanceToGoalTolerance()
   {
      return orientationDistanceToGoalTolerance.getValue();
   }

   public void setOrientationDistanceToGoalTolerance(double orientationDistanceToGoalTolerance)
   {
      this.orientationDistanceToGoalTolerance.setValue(orientationDistanceToGoalTolerance);
   }
}
