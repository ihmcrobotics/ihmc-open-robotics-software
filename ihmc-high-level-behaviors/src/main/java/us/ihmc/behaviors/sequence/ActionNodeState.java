package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.crdt.CRDTUnidirectionalDoubleArray;
import us.ihmc.communication.crdt.CRDTUnidirectionalInteger;
import us.ihmc.communication.crdt.CRDTUnidirectionalOneDoFJointTrajectoryList;
import us.ihmc.communication.crdt.CRDTUnidirectionalPose3D;
import us.ihmc.communication.crdt.CRDTUnidirectionalSE3Trajectory;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public abstract class ActionNodeState<D extends ActionNodeDefinition> extends BehaviorTreeNodeState<D>
{
   public static final int SUPPORTED_NUMBER_OF_JOINTS = 7;

   private final CRDTUnidirectionalBoolean isNextForExecution;
   private final CRDTUnidirectionalInteger concurrencyRank;
   private final CRDTUnidirectionalBoolean canExecute;
   private final CRDTUnidirectionalBoolean isExecuting;
   private final CRDTUnidirectionalBoolean failed;
   private final CRDTUnidirectionalDouble nominalExecutionDuration;
   private final CRDTUnidirectionalDouble elapsedExecutionTime;
   private final CRDTUnidirectionalSE3Trajectory commandedTrajectory;
   private final CRDTUnidirectionalPose3D currentPose;
   private final CRDTUnidirectionalOneDoFJointTrajectoryList commandedJointTrajectories;
   private final CRDTUnidirectionalDoubleArray currentJointAngles;
   private final CRDTUnidirectionalDouble positionDistanceToGoalTolerance;
   private final CRDTUnidirectionalDouble orientationDistanceToGoalTolerance;

   /** The index is not CRDT synced because it's a simple local calculation. */
   private int actionIndex = -1;
   private ActionNodeState<?> executeAfterNode;

   public ActionNodeState(long id, D definition, CRDTInfo crdtInfo)
   {
      super(id, definition, crdtInfo);

      isNextForExecution = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      concurrencyRank = new CRDTUnidirectionalInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 1);
      canExecute = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, true);
      isExecuting = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      failed = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      nominalExecutionDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      elapsedExecutionTime = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      commandedTrajectory = new CRDTUnidirectionalSE3Trajectory(ROS2ActorDesignation.ROBOT, crdtInfo);
      currentPose = new CRDTUnidirectionalPose3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      commandedJointTrajectories = new CRDTUnidirectionalOneDoFJointTrajectoryList(ROS2ActorDesignation.ROBOT, crdtInfo);
      currentJointAngles = new CRDTUnidirectionalDoubleArray(ROS2ActorDesignation.ROBOT, crdtInfo, SUPPORTED_NUMBER_OF_JOINTS);
      positionDistanceToGoalTolerance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      orientationDistanceToGoalTolerance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
   }

   public void toMessage(ActionNodeStateMessage message)
   {
      super.toMessage(message.getState());

      message.setIsNextForExecution(isNextForExecution.toMessage());
      message.setConcurrencyRank(concurrencyRank.toMessage());
      message.setCanExecute(canExecute.toMessage());
      message.setIsExecuting(isExecuting.toMessage());
      message.setFailed(failed.toMessage());
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

      isNextForExecution.fromMessage(message.getIsNextForExecution());
      concurrencyRank.fromMessage(message.getConcurrencyRank());
      canExecute.fromMessage(message.getCanExecute());
      isExecuting.fromMessage(message.getIsExecuting());
      failed.fromMessage(message.getFailed());
      nominalExecutionDuration.fromMessage(message.getNominalExecutionDuration());
      elapsedExecutionTime.fromMessage(message.getElapsedExecutionTime());
      commandedTrajectory.fromMessage(message.getCommandedTrajectory());
      currentPose.fromMessage(message.getCurrentPose());
      commandedJointTrajectories.fromMessage(message.getCommandedJointTrajectories());
      currentJointAngles.fromMessage(message.getCurrentJointAngles());
      positionDistanceToGoalTolerance.fromMessage(message.getPositionDistanceToGoalTolerance());
      orientationDistanceToGoalTolerance.fromMessage(message.getOrientationDistanceToGoalTolerance());
   }

   public void setActionIndex(int actionIndex)
   {
      this.actionIndex = actionIndex;
   }

   public int getActionIndex()
   {
      return actionIndex;
   }

   public void setIsNextForExecution(boolean isNextForExecution)
   {
      this.isNextForExecution.setValue(isNextForExecution);
   }

   public boolean getIsNextForExecution()
   {
      return isNextForExecution.getValue();
   }

   public void setConcurrencyRank(int concurrencyRank)
   {
      this.concurrencyRank.setValue(concurrencyRank);
   }

   public int getConcurrencyRank()
   {
      return concurrencyRank.getValue();
   }

   public boolean getIsToBeExecutedConcurrently()
   {
      return concurrencyRank.getValue() > 1;
   }

   public void setCanExecute(boolean canExecute)
   {
      this.canExecute.setValue(canExecute);
   }

   /** @return whether this action is valid for execution. This is checked before triggering the action. */
   public boolean getCanExecute()
   {
      return canExecute.getValue();
   }

   /** Set from within {@link ActionNodeExecutor#updateCurrentlyExecuting} only. */
   public void setIsExecuting(boolean isExecuting)
   {
      this.isExecuting.setValue(isExecuting);
   }

   public void setFailed(boolean failed)
   {
      this.failed.setValue(failed);
   }

   public boolean getFailed()
   {
      return failed.getValue();
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

   public CRDTUnidirectionalSE3Trajectory getCommandedTrajectory()
   {
      return commandedTrajectory;
   }

   public CRDTUnidirectionalPose3D getCurrentPose()
   {
      return currentPose;
   }

   public CRDTUnidirectionalOneDoFJointTrajectoryList getCommandedJointTrajectories()
   {
      return commandedJointTrajectories;
   }

   public CRDTUnidirectionalDoubleArray getCurrentJointAngles()
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

   /** Should return a precalculated value from {@link ActionNodeExecutor#updateCurrentlyExecuting} */
   public boolean getIsExecuting()
   {
      return isExecuting.getValue();
   }

   public void setExecuteAfterNode(ActionNodeState<?> executeAfterNode)
   {
      this.executeAfterNode = executeAfterNode;
   }

   public boolean getEffectivelyExecuteAfterBeginning()
   {
      return getDefinition().getExecuteAfterBeginning() || (actionIndex == 0 && getDefinition().getExecuteAfterPrevious());
   }

   public ActionNodeState<?> getExecuteAfterNode()
   {
      return executeAfterNode;
   }
}
