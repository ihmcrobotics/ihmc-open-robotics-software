package us.ihmc.avatar;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.yoVariables.variable.YoLong;

public class PerceptionTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver threadResolver;
   private final CrossRobotCommandResolver masterResolver;

   private final AvatarControllerThreadInterface thread;

   private final long divisor;
   private final ThreadTimer timer;
   private final YoLong ticksBehindScheduled;

   private final List<Runnable> taskThreadRunnables = new ArrayList<>();
   private final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

   public PerceptionTask(String prefix, AvatarControllerThreadInterface thread, long divisor, double schedulerDt, FullHumanoidRobotModel masterFullRobotModel)
   {
      super(divisor);
      this.divisor = divisor;
      this.thread = thread;

      threadResolver = new CrossRobotCommandResolver(thread.getFullRobotModel());
      masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);

      timer = new ThreadTimer(prefix, schedulerDt * divisor, thread.getYoVariableRegistry());
      ticksBehindScheduled = new YoLong(prefix + "TicksBehindScheduled", thread.getYoVariableRegistry());
   }

   @Override
   protected void execute()
   {
      timer.start();
      long schedulerTick = thread.getHumanoidRobotContextData().getSchedulerTick();
      ticksBehindScheduled.set(schedulerTick - timer.getTickCount() * divisor);
      thread.run();
      runAll(taskThreadRunnables);
      timer.stop();
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
      masterResolver.resolveHumanoidRobotContextDataPerception(thread.getHumanoidRobotContextData(), masterContext);
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      threadResolver.resolveHumanoidRobotContextDataScheduler(masterContext, thread.getHumanoidRobotContextData());
      threadResolver.resolveHumanoidRobotContextDataEstimator(masterContext, thread.getHumanoidRobotContextData());
   }

   @Override
   public void addCallbackPostTask(Runnable callback)
   {
      taskThreadRunnables.add(callback);
   }

   @Override
   public void addRunnableOnSchedulerThread(Runnable runnable)
   {
      schedulerThreadRunnables.add(runnable);
   }

}
