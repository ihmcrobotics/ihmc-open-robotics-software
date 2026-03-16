package us.ihmc.avatar;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;
import java.util.List;

public class StepGeneratorTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver controllerResolver;

   private final AvatarStepGeneratorThread thread;

   private final int divisor;
   private final ThreadTimer timer;
   private final YoLong ticksBehindScheduled;

   protected final List<Runnable> postControllerCallbacks = new ArrayList<>();
   protected final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

   public StepGeneratorTask(String prefix, AvatarStepGeneratorThread thread, int divisor, double schedulerDt)
   {
      super(divisor);

      this.divisor = divisor;
      this.thread = thread;

      controllerResolver = new CrossRobotCommandResolver(thread.getFullRobotModel());

      //      String prefix = "Controller";
      timer = new ThreadTimer(prefix, schedulerDt * divisor, thread.getYoVariableRegistry());
      ticksBehindScheduled = new YoLong(prefix + "TicksBehindScheduled", thread.getYoVariableRegistry());
   }

   @Override
   protected boolean initialize()
   {
      // For when the task gets reset, so we can observe when it gets triggered.
      timer.reset();
      ticksBehindScheduled.set(0);
      return super.initialize();
   }

   @Override
   protected void execute()
   {
      timer.start();
      long schedulerTick = thread.getHumanoidRobotContextData().getSchedulerTick();
      ticksBehindScheduled.set(schedulerTick - timer.getTickCount() * divisor);
      thread.run();
      runAll(postControllerCallbacks);
      timer.stop();
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      controllerResolver.resolveHumanoidRobotContextDataScheduler(masterContext, thread.getHumanoidRobotContextData());
      controllerResolver.resolveHumanoidRobotContextDataEstimator(masterContext, thread.getHumanoidRobotContextData());
   }

   @Override
   public void addCallbackPostTask(Runnable runnable)
   {
      postControllerCallbacks.add(runnable);
   }

   @Override
   public void addRunnableOnSchedulerThread(Runnable runnable)
   {
      schedulerThreadRunnables.add(runnable);
   }
}
