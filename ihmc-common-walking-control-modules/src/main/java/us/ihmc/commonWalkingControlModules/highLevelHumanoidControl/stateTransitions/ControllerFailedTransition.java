package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions;

import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ControllerFailedTransition implements StateTransitionCondition
{
   private final YoBoolean controllerFailed;
   private final BooleanProvider isRobotOffSupport;
   private final HighLevelControllerName nextStateEnum;

   public ControllerFailedTransition(YoBoolean controllerFailed, BooleanProvider isRobotOffSupport, HighLevelControllerName nextStateEnum)
   {
      this.controllerFailed = controllerFailed;
      this.isRobotOffSupport = isRobotOffSupport;
      this.nextStateEnum = nextStateEnum;
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
      boolean controllerFailed = this.controllerFailed.getBooleanValue();
      boolean shouldTransition = (isRobotOffSupport.getValue() && nextStateEnum.equals(HighLevelControllerName.FALLING_STATE)) ||
                                 (!isRobotOffSupport.getValue() && !nextStateEnum.equals(HighLevelControllerName.FALLING_STATE));

      if (controllerFailed && shouldTransition)
         this.controllerFailed.set(false);

      return controllerFailed && shouldTransition;
   }
}
