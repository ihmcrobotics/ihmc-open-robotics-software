package us.ihmc.avatar;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.concurrent.ConcurrentCopier;
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

   /**
    * Lets the master thread pick up this task's freshly computed desired joint outputs as soon as
    * {@link #execute()} finishes, instead of waiting for the barrier scheduler to bubble them up via
    * {@link #updateMasterContext(HumanoidRobotContextData)}, which only happens on this task's own
    * next release (i.e. up to one full control period later). Single producer (this task's thread),
    * single consumer (the master thread), so a lock-free/garbage-free {@link ConcurrentCopier} is safe.
    */
   private final ConcurrentCopier<LowLevelOneDoFJointDesiredDataHolder> fastJointDesiredOutputCopier = new ConcurrentCopier<>(LowLevelOneDoFJointDesiredDataHolder::new);

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

      // Publish the freshly computed desired joint outputs immediately, resolved into the master
      // robot model's joint identities, so the master thread doesn't have to wait for this task's
      // next release to see them (see fastJointDesiredOutputCopier).
      masterResolver.resolveLowLevelOneDoFJointDesiredDataHolder(controllerThread.getHumanoidRobotContextData().getJointDesiredOutputList(),
                                                                  fastJointDesiredOutputCopier.getCopyForWriting());
      fastJointDesiredOutputCopier.commit();

      runAll(postTaskCallbacks);
      timer.stop();
   }

   /**
    * Returns the fast, low-latency channel for this task's desired joint outputs. See
    * {@link #fastJointDesiredOutputCopier} for why this exists alongside the normal master-context
    * hand-off.
    */
   public ConcurrentCopier<LowLevelOneDoFJointDesiredDataHolder> getFastJointDesiredOutputCopier()
   {
      return fastJointDesiredOutputCopier;
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
}
