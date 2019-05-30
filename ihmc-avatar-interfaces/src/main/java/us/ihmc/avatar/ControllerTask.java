package us.ihmc.avatar;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ThreadTimer;

public class ControllerTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver controllerResolver;
   private final CrossRobotCommandResolver masterResolver;

   private final AvatarControllerThread controllerThread;

   private final ThreadTimer timer;

   private final List<Runnable> taskThreadRunnables = new ArrayList<>();
   private final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

   public ControllerTask(AvatarControllerThread controllerThread, long divisor, FullHumanoidRobotModel masterFullRobotModel)
   {
      super(divisor);
      this.controllerThread = controllerThread;

      controllerResolver = new CrossRobotCommandResolver(controllerThread.getFullRobotModel());
      masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);

      timer = new ThreadTimer("Controller", controllerThread.getYoVariableRegistry());
   }

   @Override
   protected void execute()
   {
      timer.start();
      controllerThread.run();
      controllerThread.write();
      runAll(taskThreadRunnables);
      timer.stop();
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
      masterResolver.resolveHumanoidRobotContextDataController(controllerThread.getHumanoidRobotContextData(), masterContext);
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      controllerResolver.resolveHumanoidRobotContextDataScheduler(masterContext, controllerThread.getHumanoidRobotContextData());
      controllerResolver.resolveHumanoidRobotContextDataEstimator(masterContext, controllerThread.getHumanoidRobotContextData());
   }

   @Override
   public void addRunnableOnTaskThread(Runnable runnable)
   {
      taskThreadRunnables.add(runnable);
   }

   @Override
   public void addRunnableOnSchedulerThread(Runnable runnable)
   {
      schedulerThreadRunnables.add(runnable);
   }

}
