package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.EnumMap;

public class RequestedControllerStateTransitionFactory<E extends Enum<E>> implements ControllerStateTransitionFactory<E>
{
   private final YoEnum<E> requestedControlState;
   private final E stateToAttachEnum;
   private final E nextStateEnum;

   private StateTransition<E> stateTransition;

   /**
    * This transition will transition the robot from its current state into a requested state if the yo enum matches the next state enum.
    *
    * @param requestedControlState yo variable used to request a state change.
    * @param stateToAttachEnum state to check if the next state has been requested.
    * @param nextStateEnum state to transition to on request.
    */
   public RequestedControllerStateTransitionFactory(YoEnum<E> requestedControlState, E stateToAttachEnum, E nextStateEnum)
   {
      this.requestedControlState = requestedControlState;
      this.stateToAttachEnum = stateToAttachEnum;
      this.nextStateEnum = nextStateEnum;
   }

   /** {@inheritDoc} */
   @Override
   public StateTransition<E> getOrCreateStateTransition(EnumMap<E, ? extends FinishableState<E>> stateMap, HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                        YoVariableRegistry parentRegistry)
   {
      if (stateTransition == null)
         stateTransition = StateMachineTools.buildRequestableStateTransition(requestedControlState, nextStateEnum);

      return stateTransition;
   }

   /** {@inheritDoc} */
   @Override
   public E getStateToAttachEnum()
   {
      return stateToAttachEnum;
   }
}
