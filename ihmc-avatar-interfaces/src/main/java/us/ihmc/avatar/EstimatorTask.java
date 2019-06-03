package us.ihmc.avatar;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ThreadTimer;

public class EstimatorTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver estimatorResolver;
   private final CrossRobotCommandResolver masterResolver;

   private final AvatarEstimatorThread estimatorThread;

   private final ThreadTimer timer;

   private final List<Runnable> taskThreadRunnables = new ArrayList<>();
   private final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

   // This is needed for the single threaded mode as the master context will be updated after the first execute call.
   private boolean masterContextUpdated = false;

   public EstimatorTask(AvatarEstimatorThread estimatorThread, long divisor, FullHumanoidRobotModel masterFullRobotModel)
   {
      super(divisor);
      this.estimatorThread = estimatorThread;

      estimatorResolver = new CrossRobotCommandResolver(estimatorThread.getFullRobotModel());
      masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);

      timer = new ThreadTimer("Estimator", estimatorThread.getYoVariableRegistry());
   }

   @Override
   protected void execute()
   {
      timer.start();
      if (masterContextUpdated)
      {
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
