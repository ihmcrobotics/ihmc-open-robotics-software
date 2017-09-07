package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;

import java.util.EnumMap;

public interface ControllerStateTransitionFactory<E extends Enum<E>>
{
   StateTransition<E> getOrCreateStateTransition(EnumMap<E, ? extends FinishableState<E>> stateMap);

   E getStateToAttachEnum();
}
