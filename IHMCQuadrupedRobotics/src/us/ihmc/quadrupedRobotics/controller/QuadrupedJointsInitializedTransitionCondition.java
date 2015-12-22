package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;

public class QuadrupedJointsInitializedTransitionCondition implements StateTransitionCondition
{
   private final EnumYoVariable<QuadrupedControllerState> requestedState;
   private final QuadrupedJointInitializer controller;

   private final QuadrupedControllerState nextState;

   public QuadrupedJointsInitializedTransitionCondition(EnumYoVariable<QuadrupedControllerState> requestedState,
         QuadrupedJointInitializer controller, QuadrupedControllerState nextState)
   {
      this.requestedState = requestedState;
      this.controller = controller;
      this.nextState = nextState;
   }

   @Override
   public boolean checkCondition()
   {
      // Transition when all joints have been initialized.
      return requestedState.getEnumValue() == nextState && controller.allJointsInitialized();
   }
}
