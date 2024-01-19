package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.crdt.CRDTUnidirectionalPose3D;
import us.ihmc.communication.crdt.CRDTUnidirectionalSE3Trajectory;
import us.ihmc.communication.crdt.CRDTUnidirectionalSpatialVector;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public abstract class ActionNodeState<D extends ActionNodeDefinition> extends BehaviorTreeNodeState<D>
{
   /** The index is not CRDT synced because it's a simple local calculation. */
   private int actionIndex = -1;
   private final CRDTUnidirectionalBoolean isNextForExecution;
   private final CRDTUnidirectionalBoolean isToBeExecutedConcurrently;
   private final CRDTUnidirectionalBoolean canExecute;
   private final CRDTUnidirectionalBoolean isExecuting;
   private final CRDTUnidirectionalBoolean failed;
   private final CRDTUnidirectionalDouble nominalExecutionDuration;
   private final CRDTUnidirectionalDouble elapsedExecutionTime;
   private final CRDTUnidirectionalSE3Trajectory desiredTrajectory;
   private final CRDTUnidirectionalPose3D currentPose;
   private final CRDTUnidirectionalSpatialVector currentTwist;
   private final CRDTUnidirectionalDouble positionDistanceToGoalTolerance;
   private final CRDTUnidirectionalDouble orientationDistanceToGoalTolerance;

   public ActionNodeState(long id, D definition, CRDTInfo crdtInfo)
   {
      super(id, definition, crdtInfo);

      isNextForExecution = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      isToBeExecutedConcurrently = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      canExecute = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, true);
      isExecuting = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      failed = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      nominalExecutionDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      elapsedExecutionTime = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      desiredTrajectory = new CRDTUnidirectionalSE3Trajectory(ROS2ActorDesignation.ROBOT, crdtInfo);
      currentPose = new CRDTUnidirectionalPose3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      currentTwist = new CRDTUnidirectionalSpatialVector(ROS2ActorDesignation.ROBOT, crdtInfo);
      positionDistanceToGoalTolerance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      orientationDistanceToGoalTolerance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
   }

   public void toMessage(ActionNodeStateMessage message)
   {
      super.toMessage(message.getState());

      message.setIsNextForExecution(isNextForExecution.toMessage());
      message.setIsToBeExecutedConcurrently(isToBeExecutedConcurrently.toMessage());
      message.setCanExecute(canExecute.toMessage());
      message.setIsExecuting(isExecuting.toMessage());
      message.setFailed(failed.toMessage());
      message.setNominalExecutionDuration(nominalExecutionDuration.toMessage());
      message.setElapsedExecutionTime(elapsedExecutionTime.toMessage());
      desiredTrajectory.toMessage(message.getDesiredTrajectory());
      currentPose.toMessage(message.getCurrentPose());
      currentTwist.toMessage(message.getCurrentTwist());
      message.setPositionDistanceToGoalTolerance(positionDistanceToGoalTolerance.toMessage());
      message.setOrientationDistanceToGoalTolerance(orientationDistanceToGoalTolerance.toMessage());
   }

   public void fromMessage(ActionNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      isNextForExecution.fromMessage(message.getIsNextForExecution());
      isToBeExecutedConcurrently.fromMessage(message.getIsToBeExecutedConcurrently());
      canExecute.fromMessage(message.getCanExecute());
      isExecuting.fromMessage(message.getIsExecuting());
      failed.fromMessage(message.getFailed());
      nominalExecutionDuration.fromMessage(message.getNominalExecutionDuration());
      elapsedExecutionTime.fromMessage(message.getElapsedExecutionTime());
      desiredTrajectory.fromMessage(message.getDesiredTrajectory());
      currentPose.fromMessage(message.getCurrentPose());
      currentTwist.fromMessage(message.getCurrentTwist());
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

   public void setIsToBeExecutedConcurrently(boolean isToBeExecutedConcurrently)
   {
      this.isToBeExecutedConcurrently.setValue(isToBeExecutedConcurrently);
   }

   public boolean getIsToBeExecutedConcurrently()
   {
      return isToBeExecutedConcurrently.getValue();
   }

   public void setCanExecute(boolean canExecute)
   {
      this.canExecute.setValue(canExecute);
   }

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

   public CRDTUnidirectionalSE3Trajectory getDesiredTrajectory()
   {
      return desiredTrajectory;
   }

   public CRDTUnidirectionalPose3D getCurrentPose()
   {
      return currentPose;
   }

   public CRDTUnidirectionalSpatialVector getCurrentTwist()
   {
      return currentTwist;
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
}
