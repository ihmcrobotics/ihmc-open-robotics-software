package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public abstract class ActionNodeState<D extends ActionNodeDefinition> extends BehaviorTreeNodeState<D>
{
   /** The index is not CRDT synced because it's a simple local calculation. */
   private int actionIndex = -1;
   private final CRDTUnidirectionalBoolean isNextForExecution;
   private final CRDTUnidirectionalBoolean isToBeExecutedConcurrently;
   private final CRDTUnidirectionalBoolean canExecute;
   private final CRDTUnidirectionalBoolean isExecuting;
   private final CRDTUnidirectionalDouble nominalExecutionDuration;
   private final CRDTUnidirectionalDouble elapsedExecutionTime;
   private final CRDTUnidirectionalDouble currentPositionDistanceToGoal;
   private final CRDTUnidirectionalDouble startPositionDistanceToGoal;
   private final CRDTUnidirectionalDouble positionDistanceToGoalTolerance;
   private final CRDTUnidirectionalDouble currentOrientationDistanceToGoal;
   private final CRDTUnidirectionalDouble startOrientationDistanceToGoal;
   private final CRDTUnidirectionalDouble orientationDistanceToGoalTolerance;

   public ActionNodeState(long id, D definition, CRDTInfo crdtInfo)
   {
      super(id, definition, crdtInfo);

      isNextForExecution = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      isToBeExecutedConcurrently = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      canExecute = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, true);
      isExecuting = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      nominalExecutionDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      elapsedExecutionTime = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      currentPositionDistanceToGoal = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      startPositionDistanceToGoal = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      positionDistanceToGoalTolerance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      currentOrientationDistanceToGoal = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      startOrientationDistanceToGoal = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      orientationDistanceToGoalTolerance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
   }

   public void toMessage(ActionNodeStateMessage message)
   {
      super.toMessage(message.getState());

      message.setIsNextForExecution(isNextForExecution.toMessage());
      message.setIsToBeExecutedConcurrently(isToBeExecutedConcurrently.toMessage());
      message.setCanExecute(canExecute.toMessage());
      message.setIsExecuting(isExecuting.toMessage());
      message.setNominalExecutionDuration(nominalExecutionDuration.toMessage());
      message.setElapsedExecutionTime(elapsedExecutionTime.toMessage());
      message.setNominalExecutionDuration(nominalExecutionDuration.toMessage());
      message.setElapsedExecutionTime(elapsedExecutionTime.toMessage());
      message.setCurrentPositionDistanceToGoal(currentPositionDistanceToGoal.toMessage());
      message.setStartPositionDistanceToGoal(startPositionDistanceToGoal.toMessage());
      message.setPositionDistanceToGoalTolerance(positionDistanceToGoalTolerance.toMessage());
      message.setCurrentOrientationDistanceToGoal(currentOrientationDistanceToGoal.toMessage());
      message.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal.toMessage());
      message.setOrientationDistanceToGoalTolerance(orientationDistanceToGoalTolerance.toMessage());
   }

   public void fromMessage(ActionNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      isNextForExecution.fromMessage(message.getIsNextForExecution());
      isToBeExecutedConcurrently.fromMessage(message.getIsToBeExecutedConcurrently());
      canExecute.fromMessage(message.getCanExecute());
      isExecuting.fromMessage(message.getIsExecuting());
      nominalExecutionDuration.fromMessage(message.getNominalExecutionDuration());
      elapsedExecutionTime.fromMessage(message.getElapsedExecutionTime());
      currentPositionDistanceToGoal.fromMessage(message.getCurrentPositionDistanceToGoal());
      startPositionDistanceToGoal.fromMessage(message.getStartPositionDistanceToGoal());
      positionDistanceToGoalTolerance.fromMessage(message.getPositionDistanceToGoalTolerance());
      currentOrientationDistanceToGoal.fromMessage(message.getCurrentOrientationDistanceToGoal());
      startOrientationDistanceToGoal.fromMessage(message.getStartOrientationDistanceToGoal());
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

   public void setIsExecuting(boolean isExecuting)
   {
      this.isExecuting.setValue(isExecuting);
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

   public double getCurrentPositionDistanceToGoal()
   {
      return currentPositionDistanceToGoal.getValue();
   }

   public void setCurrentPositionDistanceToGoal(double currentPositionDistanceToGoal)
   {
      this.currentPositionDistanceToGoal.setValue(currentPositionDistanceToGoal);
   }

   public double getStartPositionDistanceToGoal()
   {
      return startPositionDistanceToGoal.getValue();
   }

   public void setStartPositionDistanceToGoal(double startPositionDistanceToGoal)
   {
      this.startPositionDistanceToGoal.setValue(startPositionDistanceToGoal);
   }

   public double getPositionDistanceToGoalTolerance()
   {
      return positionDistanceToGoalTolerance.getValue();
   }

   public void setPositionDistanceToGoalTolerance(double positionDistanceToGoalTolerance)
   {
      this.positionDistanceToGoalTolerance.setValue(positionDistanceToGoalTolerance);
   }

   public double getCurrentOrientationDistanceToGoal()
   {
      return currentOrientationDistanceToGoal.getValue();
   }

   public void setCurrentOrientationDistanceToGoal(double currentOrientationDistanceToGoal)
   {
      this.currentOrientationDistanceToGoal.setValue(currentOrientationDistanceToGoal);
   }

   public double getStartOrientationDistanceToGoal()
   {
      return startOrientationDistanceToGoal.getValue();
   }

   public void setStartOrientationDistanceToGoal(double startOrientationDistanceToGoal)
   {
      this.startOrientationDistanceToGoal.setValue(startOrientationDistanceToGoal);
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
