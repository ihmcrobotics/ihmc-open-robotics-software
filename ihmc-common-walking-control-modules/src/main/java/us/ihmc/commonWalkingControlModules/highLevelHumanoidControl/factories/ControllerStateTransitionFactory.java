package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.EnumMap;

public interface ControllerStateTransitionFactory<E extends Enum<E>>
{
   /**
    * Gets the new transition. If it has not been created previously, this creates it, and then returns it.
    */
   StateTransition<E> getOrCreateStateTransition(EnumMap<E, ? extends FinishableState<E>> stateMap, HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                 YoVariableRegistry parentRegistry);

   /**
    * The state in which this transition is to be checked.
    */
   E getStateToAttachEnum();
}
