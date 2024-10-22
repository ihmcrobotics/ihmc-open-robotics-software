package us.ihmc.avatar;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;
import java.util.List;

public class WholeBodyControllerCoreTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver wholeBodyControllerCoreResolver;
   private final CrossRobotCommandResolver masterResolver;
   private final ThreadTimer timer;
   private final YoLong ticksBehindScheduled;

   protected final List<Runnable> postWholeBodyControllerCoreCallbacks = new ArrayList<>();
   protected final List<Runnable> getSchedulerThreadRunnables = new ArrayList<>();

   private final AvatarControllerThreadInterface wholeBodyControllerCoreThread;
   private final long divisor;
   protected final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

   public WholeBodyControllerCoreTask(String prefix,
                                      AvatarControllerThreadInterface wholeBodyControllerThread,
                                      long divisor,
                                      double schedulerDt,
                                      FullHumanoidRobotModel masterFullRobotModel)
   {
      //      super(prefix, wholeBodyControllerThread, divisor, schedulerDt, masterFullRobotModel);
      super(divisor);
      this.divisor = divisor;
      this.wholeBodyControllerCoreThread = wholeBodyControllerThread;

      wholeBodyControllerCoreResolver = new CrossRobotCommandResolver(wholeBodyControllerCoreThread.getFullRobotModel());
      masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);

      timer = new ThreadTimer(prefix, schedulerDt * divisor, wholeBodyControllerThread.getYoVariableRegistry());
      ticksBehindScheduled = new YoLong(prefix + "TicksBehindScheduled", wholeBodyControllerThread.getYoVariableRegistry());
   }

   @Override
   protected boolean initialize()
   {
      //For when the task gets rest, so we can observe when it gets triggered.
      timer.reset();
      ticksBehindScheduled.set(0);
      return super.initialize();
   }

   @Override
   protected void execute()
   {
      timer.start();
      long schedulerTick = wholeBodyControllerCoreThread.getHumanoidRobotContextData().getSchedulerTick();
      ticksBehindScheduled.set(schedulerTick - timer.getTickCount() * divisor);
      wholeBodyControllerCoreThread.run();
      runAll(postWholeBodyControllerCoreCallbacks);
      timer.stop();
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
      masterResolver.resolveHumanoidRobotContextDataWholeBodyControllerCore(wholeBodyControllerCoreThread.getHumanoidRobotContextData(), masterContext);
//      masterResolver.resolveHumanoidRobotContextDataWholeBodyControllerCoreFull(wholeBodyControllerCoreThread.getHumanoidRobotContextData(), masterContext);
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      wholeBodyControllerCoreResolver.resolveHumanoidRobotContextDataScheduler(masterContext, wholeBodyControllerCoreThread.getHumanoidRobotContextData());
      wholeBodyControllerCoreResolver.resolveHumanoidRobotContextDataEstimator(masterContext, wholeBodyControllerCoreThread.getHumanoidRobotContextData());
      wholeBodyControllerCoreResolver.resolveHumanoidRobotContextDataController(masterContext, wholeBodyControllerCoreThread.getHumanoidRobotContextData());
      wholeBodyControllerCoreResolver.resolveHumanoidRobotContextDataWholeBodyControllerCoreFull(masterContext, wholeBodyControllerCoreThread.getHumanoidRobotContextData());
   }

   @Override
   public void addCallbackPostTask(Runnable runnable) {postWholeBodyControllerCoreCallbacks.add(runnable); }

   @Override
   public void addRunnableOnSchedulerThread(Runnable runnable) { schedulerThreadRunnables.add(runnable); }
}
