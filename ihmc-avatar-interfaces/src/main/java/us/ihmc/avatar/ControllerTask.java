package us.ihmc.avatar;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.yoVariables.variable.YoLong;

public class ControllerTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver controllerResolver;
   private final CrossRobotCommandResolver masterResolver;

   private final AvatarControllerThreadInterface controllerThread;

   private final ThreadTimer timer;
   private final YoLong ticksBehindScheduled;

   protected final List<Runnable> postControllerCallbacks = new ArrayList<>();
   protected final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

   private final double schedulerDt;

   public ControllerTask(String prefix,
                         AvatarControllerThreadInterface controllerThread,
                         double schedulerDt,
                         FullHumanoidRobotModel masterFullRobotModel)
   {
      super((int) Math.round(controllerThread.getCurrentDT() / schedulerDt));
      this.controllerThread = controllerThread;
      this.schedulerDt = schedulerDt;

      controllerResolver = new CrossRobotCommandResolver(controllerThread.getFullRobotModel());
      masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);

      //      String prefix = "Controller";
      timer = new ThreadTimer(prefix, schedulerDt * (int) Math.round(controllerThread.getCurrentDT() / schedulerDt), controllerThread.getYoVariableRegistry());
      ticksBehindScheduled = new YoLong(prefix + "TicksBehindScheduled", controllerThread.getYoVariableRegistry());
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
      long oldDivisor = getDivisor();
      long schedulerTick = controllerThread.getHumanoidRobotContextData().getSchedulerTick();
      int divisor = (int) Math.round(controllerThread.getCurrentDT() / schedulerDt);
      if (divisor != oldDivisor)
         setDivisor(divisor);
      ticksBehindScheduled.set(schedulerTick - timer.getTickCount() * oldDivisor);
      controllerThread.run();
      runAll(postControllerCallbacks);
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
