package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ControllerFailedTransition implements StateTransitionCondition
{
   private final YoBoolean controllerFailed;

   public ControllerFailedTransition(HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry registry)
   {
      controllerFailed = controllerToolbox.getControllerFailedBoolean();
   }

   private boolean pollControllerFailed()
   {
      boolean controllerFailed = this.controllerFailed.getBooleanValue();
      this.controllerFailed.set(false);
      return controllerFailed;
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      return pollControllerFailed();
   }
}
