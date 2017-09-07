package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.EnumMap;

public class RequestedControllerStateTransitionFactory<E extends Enum<E>> implements ControllerStateTransitionFactory<E>
{
   private final YoEnum<E> requestedControlState;
   private final E stateToAttach;
   private final E nextStateEnum;

   private StateTransition<E> stateTransition;

   public RequestedControllerStateTransitionFactory(YoEnum<E> requestedControlState, E stateToAttach, E nextStateEnum)
   {
      this.requestedControlState = requestedControlState;
      this.stateToAttach = stateToAttach;
      this.nextStateEnum = nextStateEnum;
   }

   @Override
   public StateTransition<E> getOrCreateStateTransition(EnumMap<E, ? extends FinishableState<E>> stateMap)
   {
      if (stateTransition == null)
         stateTransition = StateMachineTools.buildRequestableStateTransition(requestedControlState, nextStateEnum);

      return stateTransition;
   }

   @Override
   public E getStateToAttachEnum()
   {
      return stateToAttach;
   }
}
