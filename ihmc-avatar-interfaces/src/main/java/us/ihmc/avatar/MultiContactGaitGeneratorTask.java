package us.ihmc.avatar;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class MultiContactGaitGeneratorTask extends ControllerTask
{
   private final AvatarControllerThreadInterface plannerThread;
   private final CrossRobotCommandResolver plannerResolver;

   public MultiContactGaitGeneratorTask(String prefix, AvatarControllerThreadInterface plannerThread, double schedulerDt, FullHumanoidRobotModel masterFullRobotModel)
   {
      super(prefix, plannerThread, schedulerDt, masterFullRobotModel);

      this.plannerThread = plannerThread;
      plannerResolver = new CrossRobotCommandResolver(plannerThread.getFullRobotModel());
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      super.updateLocalContext(masterContext);
      plannerResolver.resolveHumanoidRobotContextDataPlanner(masterContext, plannerThread.getHumanoidRobotContextData());
   }

}
