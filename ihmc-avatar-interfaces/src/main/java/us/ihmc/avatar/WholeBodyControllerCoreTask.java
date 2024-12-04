package us.ihmc.avatar;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotMPCContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.MPCCrossRobotCommandResolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;

import java.util.ArrayList;
import java.util.List;

public class WholeBodyControllerCoreTask extends ControllerTask
{
   protected final List<Runnable> postWholeBodyControllerCoreCallbacks = new ArrayList<>();

   protected final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

   public WholeBodyControllerCoreTask(String prefix,
                                      AvatarControllerThreadInterface wholeBodyControllerThread,
                                      long divisor,
                                      double schedulerDt,
                                      FullHumanoidRobotModel masterFullRobotModel)
   {
      super(prefix, wholeBodyControllerThread, divisor, schedulerDt, masterFullRobotModel);
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
      long schedulerTick = controllerThread.getHumanoidRobotContextData().getSchedulerTick();
      ticksBehindScheduled.set(schedulerTick - timer.getTickCount() * divisor);
      controllerThread.run();
      runAll(postWholeBodyControllerCoreCallbacks);
      timer.stop();
   }

   protected void updateMasterContext(HumanoidRobotMPCContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
      AvatarMPCWholeBodyControllerCoreThread wholeBodyControllerCoreThread = (AvatarMPCWholeBodyControllerCoreThread) controllerThread;
      MPCCrossRobotCommandResolver masterResolver = (MPCCrossRobotCommandResolver) this.masterResolver;
      masterResolver.resolveHumanoidRobotContextDataWholeBodyControllerCore(wholeBodyControllerCoreThread.getHumanoidRobotContextData(), masterContext);
   }

   protected void updateLocalContext(HumanoidRobotMPCContextData masterContext)
   {
      AvatarMPCWholeBodyControllerCoreThread wholeBodyControllerCoreThread = (AvatarMPCWholeBodyControllerCoreThread) this.controllerThread;
      MPCCrossRobotCommandResolver wholeBodyControllerCoreResolver = (MPCCrossRobotCommandResolver) this.controllerResolver;
      wholeBodyControllerCoreResolver.resolveHumanoidRobotContextDataScheduler(masterContext, wholeBodyControllerCoreThread.getHumanoidRobotContextData());
      wholeBodyControllerCoreResolver.resolveHumanoidRobotContextDataEstimator(masterContext, wholeBodyControllerCoreThread.getHumanoidRobotContextData());
      wholeBodyControllerCoreResolver.resolveHumanoidRobotContextDataController(masterContext, wholeBodyControllerCoreThread.getHumanoidRobotContextData());
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