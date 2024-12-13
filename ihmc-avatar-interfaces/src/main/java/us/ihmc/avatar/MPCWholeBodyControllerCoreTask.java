package us.ihmc.avatar;

import us.ihmc.avatar.factory.MPCHumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotMPCContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;
import java.util.List;

public class MPCWholeBodyControllerCoreTask extends MPCHumanoidRobotControlTask
{
   private final CrossRobotCommandResolver wholeBodyControllerCoreResolver;
   private final CrossRobotCommandResolver masterResolver;
   private final long divisor;
   protected final List<Runnable> schedulerThreadRunnables = new ArrayList<>();
   private final AvatarControllerThreadInterface wholeBodyControllerCoreThread;
   private final ThreadTimer timer;
   private final YoLong ticksBehindScheduled;

   protected final List<Runnable> postWholeBodyControllerCoreCallbacks = new ArrayList<>();

   public MPCWholeBodyControllerCoreTask(String prefix,
                                         AvatarControllerThreadInterface wholeBodyControllerCoreThread,
                                         long divisor,
                                         double schedulerDt,
                                         FullHumanoidRobotModel masterFullRobotModel)
   {
      super(divisor);
      this.divisor = divisor;
      this.wholeBodyControllerCoreThread = wholeBodyControllerCoreThread;

      this.wholeBodyControllerCoreResolver = new CrossRobotCommandResolver(wholeBodyControllerCoreThread.getFullRobotModel());
      this.masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);

      this.timer = new ThreadTimer(prefix, schedulerDt * divisor, wholeBodyControllerCoreThread.getYoVariableRegistry());
      ticksBehindScheduled = new YoLong(prefix + "TicksBehindScheduled", wholeBodyControllerCoreThread.getYoVariableRegistry());
   }

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

   protected void updateMasterContext(HumanoidRobotMPCContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
      //      masterResolver.resolveHumanoidRobotContextDataController(wholeBodyControllerCoreThread.getHumanoidRobotContextData(), masterContext);
   }

   protected void updateLocalContext(HumanoidRobotMPCContextData masterContext)
   {
      wholeBodyControllerCoreResolver.resolveHumanoidRobotContextDataScheduler(masterContext, wholeBodyControllerCoreThread.getHumanoidRobotContextData());
      wholeBodyControllerCoreResolver.resolveHumanoidRobotContextDataEstimator(masterContext, wholeBodyControllerCoreThread.getHumanoidRobotContextData());
   }

   @Override
   public void addCallbackPostTask(Runnable runnable)
   {
      postWholeBodyControllerCoreCallbacks.add(runnable);
   }

   @Override
   public void addRunnableOnSchedulerThread(Runnable runnable)
   {
      schedulerThreadRunnables.add(runnable);
   }
}
