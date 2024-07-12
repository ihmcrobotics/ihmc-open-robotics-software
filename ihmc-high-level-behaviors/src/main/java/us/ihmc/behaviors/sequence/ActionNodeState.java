package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusDoubleArray;
import us.ihmc.communication.crdt.CRDTStatusOneDoFJointTrajectoryList;
import us.ihmc.communication.crdt.CRDTStatusPose3D;
import us.ihmc.communication.crdt.CRDTStatusSE3Trajectory;
import us.ihmc.communication.crdt.CRDTStatusBoolean;
import us.ihmc.communication.crdt.CRDTStatusDouble;
import us.ihmc.communication.crdt.CRDTStatusInteger;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

import java.util.List;

public abstract class ActionNodeState<D extends ActionNodeDefinition> extends BehaviorTreeNodeState<D>
{
   public static final int SUPPORTED_NUMBER_OF_JOINTS = 7;

   private final D definition;

   private final CRDTStatusBoolean isNextForExecution;
   private final CRDTStatusInteger concurrencyRank;
   private final CRDTStatusBoolean canExecute;
   private final CRDTStatusBoolean isExecuting;
   private final CRDTStatusBoolean failed;
   private final CRDTStatusDouble nominalExecutionDuration;
   private final CRDTStatusDouble elapsedExecutionTime;
   private final CRDTStatusSE3Trajectory commandedTrajectory;
   private final CRDTStatusPose3D currentPose;
   private final CRDTStatusOneDoFJointTrajectoryList commandedJointTrajectories;
   private final CRDTStatusDoubleArray currentJointAngles;
   private final CRDTStatusDouble positionDistanceToGoalTolerance;
   private final CRDTStatusDouble orientationDistanceToGoalTolerance;

   /** The index is not CRDT synced because it's a simple local calculation. */
   private int actionIndex = -1;

   public ActionNodeState(long id, D definition, CRDTInfo crdtInfo)
   {
      super(id, definition, crdtInfo);

      this.definition = definition;

      isNextForExecution = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      concurrencyRank = new CRDTStatusInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 1);
      canExecute = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, true);
      isExecuting = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      failed = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
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
      boolean hasStatus = false;
      hasStatus |= isNextForExecution.pollHasStatus();
      hasStatus |= concurrencyRank.pollHasStatus();
      hasStatus |= canExecute.pollHasStatus();
      hasStatus |= isExecuting.pollHasStatus();
      hasStatus |= failed.pollHasStatus();
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

   /**
    * Gives an idea how many actions will be executing all together with this one.
    * How many actions will be started when the execute next index is set to this action.
    */
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

   /** Should return a precalculated value from {@link ActionNodeExecutor#updateCurrentlyExecuting} */
   public boolean getIsExecuting()
   {
      return isExecuting.getValue();
   }

   public int calculateExecuteAfterActionIndex(List<ActionNodeState<?>> actionStateChildren)
   {
      if (definition.getExecuteAfterBeginning().getValue())
      {
         return -1;
      }
      else if (definition.getExecuteAfterPrevious().getValue())
      {
         return actionIndex - 1;
      }
      else
      {
         return findActionToExecuteAfter(actionStateChildren).getActionIndex();
      }
   }

   /** Assumes execute after node ID matches a valid action. */
   public ActionNodeState<?> findActionToExecuteAfter(List<ActionNodeState<?>> actionStateChildren)
   {
      long executeAfterID = definition.getExecuteAfterNodeID().getValue();
      for (int j = actionIndex - 1; j >= 0; j--)
      {
         ActionNodeState<?> actionStateToCompare = actionStateChildren.get(j);
         if (actionStateToCompare.getID() == executeAfterID)
         {
            return actionStateToCompare;
         }
      }

      return null;
   }
}
