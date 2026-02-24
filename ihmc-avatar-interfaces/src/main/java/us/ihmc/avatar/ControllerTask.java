package us.ihmc.avatar;

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
   private final ThreadTimer loopTimer;
   private final YoLong ticksBehindScheduled;

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
      timer = new ThreadTimer(prefix, controllerThread::getCurrentDT, controllerThread.getYoVariableRegistry());
      loopTimer = new ThreadTimer(prefix + "Loop", controllerThread::getCurrentDT, controllerThread.getYoVariableRegistry());
      ticksBehindScheduled = new YoLong(prefix + "TicksBehindScheduled", controllerThread.getYoVariableRegistry());
   }

   @Override
   protected boolean initialize()
   {
      // For when the task gets reset, so we can observe when it gets triggered.
      timer.reset();
      loopTimer.reset();
      ticksBehindScheduled.set(0);
      return super.initialize();
   }

   @Override
   protected void execute()
   {
      loopTimer.stop();
      loopTimer.start();
      timer.start();
      long oldDivisor = getDivisor();
      long schedulerTick = controllerThread.getHumanoidRobotContextData().getSchedulerTick();
      int divisor = (int) Math.round(controllerThread.getCurrentDT() / schedulerDt);
      if (divisor != oldDivisor)
         setDivisor(divisor);
      ticksBehindScheduled.set(schedulerTick - timer.getTickCount() * oldDivisor);
      ticksBehindScheduled.set(schedulerTick - timer.getTickCount() * divisor);
      runAll(preTaskCallbacks);
      controllerThread.run();
      runAll(postTaskCallbacks);
      timer.stop();
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
      masterResolver.resolveHumanoidRobotContextDataController(controllerThread.getHumanoidRobotContextData(), masterContext);
      masterResolver.resolveHumanoidRobotContextDataPlanner(controllerThread.getHumanoidRobotContextData(), masterContext);
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      controllerResolver.resolveHumanoidRobotContextDataScheduler(masterContext, controllerThread.getHumanoidRobotContextData());
      controllerResolver.resolveHumanoidRobotContextDataEstimator(masterContext, controllerThread.getHumanoidRobotContextData());
   }
}
