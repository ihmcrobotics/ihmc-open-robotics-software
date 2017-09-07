package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;

import java.util.EnumMap;

public class FinishedControllerStateTransitionFactory<E extends Enum<E>> implements ControllerStateTransitionFactory<E>
{
   private final E stateToAttach;
   private final E currentStateEnum;
   private final E nextStateEnum;

   private StateTransition<E> stateTransition;

   public FinishedControllerStateTransitionFactory(E currentStateEnum, E nextStateEnum)
   {
      this(currentStateEnum, currentStateEnum, nextStateEnum);
   }

   public FinishedControllerStateTransitionFactory(E stateToAttach, E currentStateEnum, E nextStateEnum)
   {
      this.stateToAttach = stateToAttach;
      this.currentStateEnum = currentStateEnum;
      this.nextStateEnum = nextStateEnum;
   }

   @Override
   public StateTransition<E> getOrCreateStateTransition(EnumMap<E, ? extends FinishableState<E>> stateMap)
   {
      if (stateTransition == null)
         stateTransition = StateMachineTools.buildFinishedStateTransition(stateMap.get(currentStateEnum), nextStateEnum);

      return stateTransition;
   }

   @Override
   public E getStateToAttachEnum()
   {
      return stateToAttach;
   }
}
