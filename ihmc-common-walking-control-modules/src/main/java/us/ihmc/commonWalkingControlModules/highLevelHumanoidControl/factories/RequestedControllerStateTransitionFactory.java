package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class RequestedControllerStateTransitionFactory<E extends Enum<E>> implements ControllerStateTransitionFactory<E>
{
   private final YoEnum<E> requestedControlState;
   private final E stateToAttachEnum;
   private final E nextStateEnum;

   private StateTransition<E> stateTransition;

   /**
    * This transition will transition the robot from its current state into a requested state if the yo
    * enum matches the next state enum.
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
   public StateTransition<E> getOrCreateStateTransition(EnumMap<E, ? extends State> stateMap,
                                                        HighLevelControllerFactoryHelper controllerFactoryHelper, YoRegistry parentRegistry)
   {
      if (stateTransition == null)
      {
         stateTransition = new StateTransition<>(nextStateEnum, this::isNextStateRequested);
      }

      return stateTransition;
   }

   private boolean isNextStateRequested(double timeInCurrentState)
   {
      if (requestedControlState.getEnumValue() == nextStateEnum)
      {
         requestedControlState.set(null);
         return true;
      }
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public E getStateToAttachEnum()
   {
      return stateToAttachEnum;
   }
}
