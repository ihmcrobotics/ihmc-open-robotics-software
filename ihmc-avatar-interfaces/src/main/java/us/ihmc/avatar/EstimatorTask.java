package us.ihmc.avatar;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.yoVariables.variable.YoLong;

public class EstimatorTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver estimatorResolver;
   private final CrossRobotCommandResolver masterResolver;

   private final AvatarEstimatorThread estimatorThread;

   private final long divisor;
   private final ThreadTimer timer;
   private final YoLong ticksBehindScheduled;

   private final List<Runnable> taskThreadRunnables = new ArrayList<>();
   private final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

   // This is needed for the single threaded mode as the master context will be updated after the first execute call.
   private boolean masterContextUpdated = false;

   public EstimatorTask(AvatarEstimatorThread estimatorThread, long divisor, double schedulerDt, FullHumanoidRobotModel masterFullRobotModel)
   {
      super(divisor);
      this.divisor = divisor;
      this.estimatorThread = estimatorThread;

      estimatorResolver = new CrossRobotCommandResolver(estimatorThread.getFullRobotModel());
      masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);

      String prefix = "Estimator";
      timer = new ThreadTimer(prefix, schedulerDt * divisor, estimatorThread.getYoVariableRegistry());
      ticksBehindScheduled = new YoLong(prefix + "TicksBehindScheduled", estimatorThread.getYoVariableRegistry());
   }

   @Override
   protected void execute()
   {
      timer.start();
      if (masterContextUpdated)
      {
         long schedulerTick = estimatorThread.getHumanoidRobotContextData().getSchedulerTick();
         ticksBehindScheduled.set(schedulerTick - timer.getTickCount() * divisor);
         estimatorThread.run();
         runAll(taskThreadRunnables);
      }
      timer.stop();
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
      masterResolver.resolveHumanoidRobotContextDataEstimator(estimatorThread.getHumanoidRobotContextData(), masterContext);
      masterContextUpdated = true;
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      estimatorResolver.resolveHumanoidRobotContextDataScheduler(masterContext, estimatorThread.getHumanoidRobotContextData());
      estimatorResolver.resolveHumanoidRobotContextDataController(masterContext, estimatorThread.getHumanoidRobotContextData());
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
