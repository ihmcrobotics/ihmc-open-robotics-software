package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.EnumMap;

public class FinishedControllerStateTransitionFactory<E extends Enum<E>> implements ControllerStateTransitionFactory<E>
{
   private final E stateToAttach;
   private final E currentStateEnum;
   private final E nextStateEnum;

   private StateTransition<E> stateTransition;

   /**
    * When in the {@param currentStateEnum}, the isDone() method will be checked. If it returns true, transitions to the
    * {@param nextStateEnum} state.
    *
    * @param currentStateEnum
    * @param nextStateEnum
    */
   public FinishedControllerStateTransitionFactory(E currentStateEnum, E nextStateEnum)
   {
      this(currentStateEnum, currentStateEnum, nextStateEnum);
   }

   /**
    * State transition that checks the {@param currentStateEnum} isDone() flag when in the {@param stateToAttach} state. If {@param currentStateEnum} is finished,
    * it will then transition to the state given by {@param nextStateEnum}.
    *
    * @param stateToAttach The state in which this transition condition is added. That is, the conditions will only be checked when it is in this state.
    * @param currentStateEnum The state that is to be checked to see if it is finished.
    * @param nextStateEnum The state to transition to.
    */
   public FinishedControllerStateTransitionFactory(E stateToAttach, E currentStateEnum, E nextStateEnum)
   {
      this.stateToAttach = stateToAttach;
      this.currentStateEnum = currentStateEnum;
      this.nextStateEnum = nextStateEnum;
   }

   /** {@inheritDoc} */
   @Override
   public StateTransition<E> getOrCreateStateTransition(EnumMap<E, ? extends FinishableState<E>> stateMap, HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                        YoVariableRegistry parentRegistry)
   {
      if (stateTransition == null)
         stateTransition = StateMachineTools.buildFinishedStateTransition(stateMap.get(currentStateEnum), nextStateEnum);

      return stateTransition;
   }

   /** {@inheritDoc} */
   @Override
   public E getStateToAttachEnum()
   {
      return stateToAttach;
   }
}
