package us.ihmc.quadrupedRobotics.controller.state;

import us.ihmc.quadrupedRobotics.controller.QuadrupedJointInitializer;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;

public class QuadrupedJointsInitializedTransitionCondition implements StateTransitionCondition
{
   private final QuadrupedJointInitializer controller;

   public QuadrupedJointsInitializedTransitionCondition(QuadrupedJointInitializer controller)
   {
      this.controller = controller;
   }

   @Override
   public boolean checkCondition()
   {
      // Transition when all joints have been initialized.
      return controller.allJointsInitialized();
   }
}
