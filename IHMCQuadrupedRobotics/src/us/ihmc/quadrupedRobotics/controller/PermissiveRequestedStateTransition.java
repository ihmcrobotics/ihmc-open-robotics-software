package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;

/**
 * A state machine transition that depends only on the value of a requested
 * state variable.
 * 
 * @param <E> The enum defining the state domain.
 */
public class PermissiveRequestedStateTransition<E extends Enum<E>> implements StateTransitionCondition
{
   private final EnumYoVariable<E> requestedState;
   private final E nextStateEnum;

   public PermissiveRequestedStateTransition(final EnumYoVariable<E> requestedState,
         final E nextStateEnum)
   {
      this.requestedState = requestedState;
      this.nextStateEnum = nextStateEnum;
   }

   @Override
   public boolean checkCondition()
   {
      E req = requestedState.getEnumValue();
      return req != null && req == nextStateEnum;
   }
}
