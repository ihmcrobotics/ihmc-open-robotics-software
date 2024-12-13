package us.ihmc.avatar;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotMPCContextData;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class MPCStepGeneratorTask extends MPCControllerTask
{
   public MPCStepGeneratorTask(String prefix,
                               AvatarControllerThreadInterface controllerThread,
                               long divisor,
                               double schedulerDt,
                               FullHumanoidRobotModel masterFullRobotModel)
   {
      super(prefix, controllerThread, divisor, schedulerDt, masterFullRobotModel);
   }

   @Override
   protected void updateMasterContext(HumanoidRobotMPCContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
   }
}
