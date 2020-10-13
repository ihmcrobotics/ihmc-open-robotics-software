package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.yoVariables.registry.YoRegistry;

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
   public StateTransition<E> getOrCreateStateTransition(EnumMap<E, ? extends State> stateMap, HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                        YoRegistry parentRegistry)
   {
      if (stateTransition == null)
      { // TODO When the lambda is changed to a method reference, Gradle throws an exception when compiling the code somehow, while Eclipse/IntelliJ are both fine with it...
         stateTransition = new StateTransition<>(nextStateEnum, timeInState -> stateMap.get(currentStateEnum).isDone(timeInState));
      }

      return stateTransition;
   }

   /** {@inheritDoc} */
   @Override
   public E getStateToAttachEnum()
   {
      return stateToAttach;
   }
}
