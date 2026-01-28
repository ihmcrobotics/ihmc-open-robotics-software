package us.ihmc.avatar;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class StandingPushRecoveryControlTask extends ControllerTask
{
   public StandingPushRecoveryControlTask(String prefix, AvatarControllerThreadInterface controllerThread, long divisor, double schedulerDt, FullHumanoidRobotModel masterFullRobotModel)
   {
      super(prefix, controllerThread, divisor, schedulerDt, masterFullRobotModel);
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
   }
}
