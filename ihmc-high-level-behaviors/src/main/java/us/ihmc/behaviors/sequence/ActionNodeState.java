package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;

public abstract class ActionNodeState<D extends ActionNodeDefinition>
      extends BehaviorTreeNodeState<D>
{
   private int actionIndex = -1;
   private boolean isNextForExecution = false;
   private boolean isToBeExecutedConcurrently = false;
   private boolean canExecute = true;
   private boolean isExecuting = false;
   private double nominalExecutionDuration = Double.NaN;
   private double elapsedExecutionTime = Double.NaN;
   private double currentPositionDistanceToGoal;
   private double startPositionDistanceToGoal;
   private double positionDistanceToGoalTolerance;
   private double currentOrientationDistanceToGoal;
   private double startOrientationDistanceToGoal;
   private double orientationDistanceToGoalTolerance;

   public ActionNodeState(long id, D definition, CRDTInfo crdtInfo)
   {
      super(id, definition, crdtInfo);
   }

   public void toMessage(ActionNodeStateMessage message)
   {
      super.toMessage(message.getState());

      message.setActionIndex(actionIndex);
      message.setIsNextForExecution(isNextForExecution);
      message.setIsToBeExecutedConcurrently(isToBeExecutedConcurrently);
      message.setCanExecute(canExecute);
      message.setIsExecuting(isExecuting);
      message.setNominalExecutionDuration(nominalExecutionDuration);
      message.setElapsedExecutionTime(elapsedExecutionTime);
      message.setNominalExecutionDuration(nominalExecutionDuration);
      message.setElapsedExecutionTime(elapsedExecutionTime);
      message.setCurrentPositionDistanceToGoal(currentPositionDistanceToGoal);
      message.setStartPositionDistanceToGoal(startPositionDistanceToGoal);
      message.setPositionDistanceToGoalTolerance(positionDistanceToGoalTolerance);
      message.setCurrentOrientationDistanceToGoal(currentOrientationDistanceToGoal);
      message.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
      message.setOrientationDistanceToGoalTolerance(orientationDistanceToGoalTolerance);
   }

   public void fromMessage(ActionNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      actionIndex = message.getActionIndex();
      isNextForExecution = message.getIsNextForExecution();
      isToBeExecutedConcurrently = message.getIsToBeExecutedConcurrently();
      canExecute = message.getCanExecute();
      isExecuting = message.getIsExecuting();
      nominalExecutionDuration = message.getNominalExecutionDuration();
      elapsedExecutionTime = message.getElapsedExecutionTime();
      currentPositionDistanceToGoal = message.getCurrentPositionDistanceToGoal();
      startPositionDistanceToGoal = message.getStartPositionDistanceToGoal();
      positionDistanceToGoalTolerance = message.getPositionDistanceToGoalTolerance();
      currentOrientationDistanceToGoal = message.getCurrentOrientationDistanceToGoal();
      startOrientationDistanceToGoal = message.getStartOrientationDistanceToGoal();
      orientationDistanceToGoalTolerance = message.getOrientationDistanceToGoalTolerance();
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
      this.isNextForExecution = isNextForExecution;
      freeze();
   }

   public boolean getIsNextForExecution()
   {
      return isNextForExecution;
   }

   public void setIsToBeExecutedConcurrently(boolean isToBeExecutedConcurrently)
   {
      this.isToBeExecutedConcurrently = isToBeExecutedConcurrently;
      freeze();
   }

   public boolean getIsToBeExecutedConcurrently()
   {
      return isToBeExecutedConcurrently;
   }

   public void setCanExecute(boolean canExecute)
   {
      this.canExecute = canExecute;
      freeze();
   }

   public boolean getCanExecute()
   {
      return canExecute;
   }

   public void setIsExecuting(boolean isExecuting)
   {
      this.isExecuting = isExecuting;
      freeze();
   }

   public void setNominalExecutionDuration(double nominalExecutionDuration)
   {
      this.nominalExecutionDuration = nominalExecutionDuration;
      freeze();
   }

   public double getNominalExecutionDuration()
   {
      return nominalExecutionDuration;
   }

   public void setElapsedExecutionTime(double elapsedExecutionTime)
   {
      this.elapsedExecutionTime = elapsedExecutionTime;
      freeze();
   }

   public double getElapsedExecutionTime()
   {
      return elapsedExecutionTime;
   }

   public double getCurrentPositionDistanceToGoal()
   {
      return currentPositionDistanceToGoal;
   }

   public void setCurrentPositionDistanceToGoal(double currentPositionDistanceToGoal)
   {
      this.currentPositionDistanceToGoal = currentPositionDistanceToGoal;
   }

   public double getStartPositionDistanceToGoal()
   {
      return startPositionDistanceToGoal;
   }

   public void setStartPositionDistanceToGoal(double startPositionDistanceToGoal)
   {
      this.startPositionDistanceToGoal = startPositionDistanceToGoal;
   }

   public double getPositionDistanceToGoalTolerance()
   {
      return positionDistanceToGoalTolerance;
   }

   public void setPositionDistanceToGoalTolerance(double positionDistanceToGoalTolerance)
   {
      this.positionDistanceToGoalTolerance = positionDistanceToGoalTolerance;
   }

   public double getCurrentOrientationDistanceToGoal()
   {
      return currentOrientationDistanceToGoal;
   }

   public void setCurrentOrientationDistanceToGoal(double currentOrientationDistanceToGoal)
   {
      this.currentOrientationDistanceToGoal = currentOrientationDistanceToGoal;
   }

   public double getStartOrientationDistanceToGoal()
   {
      return startOrientationDistanceToGoal;
   }

   public void setStartOrientationDistanceToGoal(double startOrientationDistanceToGoal)
   {
      this.startOrientationDistanceToGoal = startOrientationDistanceToGoal;
   }

   public double getOrientationDistanceToGoalTolerance()
   {
      return orientationDistanceToGoalTolerance;
   }

   public void setOrientationDistanceToGoalTolerance(double orientationDistanceToGoalTolerance)
   {
      this.orientationDistanceToGoalTolerance = orientationDistanceToGoalTolerance;
   }

   /** Should return a precalculated value from {@link ActionNodeExecutor#updateCurrentlyExecuting} */
   public boolean getIsExecuting()
   {
      return isExecuting;
   }
}
