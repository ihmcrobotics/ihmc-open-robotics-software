package us.ihmc.avatar;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;
import java.util.List;

public class MultiContactGaitGeneratorTask extends HumanoidRobotControlTask
{
   private final AvatarControllerThreadInterface plannerThread;
   private final CrossRobotCommandResolver plannerResolver;

   private final int divisor;
   private final ThreadTimer timer;
   private final YoLong ticksBehindScheduled;

   private final List<Runnable> postControllerCallbacks = new ArrayList<>();
   private final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

   public MultiContactGaitGeneratorTask(String prefix, AvatarControllerThreadInterface plannerThread, int divisor, double schedulerDt)
   {
      super(divisor);

      this.divisor = divisor;
      this.plannerThread = plannerThread;
      plannerResolver = new CrossRobotCommandResolver(plannerThread.getFullRobotModel());

      timer = new ThreadTimer(prefix, schedulerDt * divisor, plannerThread.getYoVariableRegistry());
      ticksBehindScheduled = new YoLong(prefix + "TicksBehindScheduled", plannerThread.getYoVariableRegistry());
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
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      plannerResolver.resolveHumanoidRobotContextDataScheduler(masterContext, plannerThread.getHumanoidRobotContextData());
      plannerResolver.resolveHumanoidRobotContextDataEstimator(masterContext, plannerThread.getHumanoidRobotContextData());
      plannerResolver.resolveHumanoidRobotContextDataPlanner(masterContext, plannerThread.getHumanoidRobotContextData());
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

   @Override
   protected void execute()
   {
      timer.start();
      long schedulerTick = plannerThread.getHumanoidRobotContextData().getSchedulerTick();
      ticksBehindScheduled.set(schedulerTick - timer.getTickCount() * divisor);
      plannerThread.run();
      runAll(postControllerCallbacks);
      timer.stop();
   }
}
