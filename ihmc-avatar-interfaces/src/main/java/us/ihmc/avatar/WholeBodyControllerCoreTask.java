package us.ihmc.avatar;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.robotModels.FullHumanoidRobotModel;

import java.util.ArrayList;
import java.util.List;

public class WholeBodyControllerCoreTask extends ControllerTask
{

   protected final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

   public WholeBodyControllerCoreTask(String prefix, AvatarControllerThreadInterface controllerThread, long divisor, double schedulerDt, FullHumanoidRobotModel masterFullRobotModel)
   {
      super(prefix, controllerThread, divisor, schedulerDt, masterFullRobotModel);
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
//      masterResolver.resolveHumanoidRobotContextDataController(controllerCoreThread.getHumanoidRobotContextData(), masterContext);
   }
}
