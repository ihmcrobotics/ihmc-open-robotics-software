package us.ihmc.avatar.hardwareControl;

import org.apache.commons.math3.util.Precision;
import us.ihmc.avatar.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.BarrierScheduledRobotController;
import us.ihmc.avatar.factory.DisposableRobotController;
import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.avatar.factory.SingleThreadedRobotController;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class AvatarMultiThreadingManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer;

   private final YoBoolean isPaused = new YoBoolean("isPaused", registry);

   private final double schedulerDt;

   private final YoLong barrierSchedulerTime = new YoLong("barrierSchedulerComputeTime", registry);

   private final DisposableRobotController threadManager;

   private final PeriodicParameters periodicParameters;
   private final PeriodicNonRealtimeThreadScheduler threadScheduler;

   private final RealtimeThread masterThread;
   private final AvatarEstimatorThread estimatorThread;
   private final List<Runnable> estimatorSchedulerThreadRunnables = new ArrayList<>();

   private ControllerTask controllerTask;
   private StepGeneratorTask stepGeneratorTask;

   private final AvatarAffinityInterface affinity;

   private final HumanoidRobotContextData masterContext;

   private final HardwareCommunicationInterface hardwareCommunicationInterface;

   private final TimestampProvider monotonicTimeProvider;

   private final boolean useRealtimeThreads;
   private final boolean useMultiThreading;

   private final AvatarLowLevelOutputProcessor lowLevelOutputProcessor;

   public AvatarMultiThreadingManager(String prefix,
                                      DRCRobotModel robotModel,
                                      HumanoidRobotContextData masterContext,
                                      FullHumanoidRobotModel masterFullRobotModel,
                                      HardwareCommunicationInterface hardwareCommunicationInterface,
                                      AvatarLowLevelOutputProcessor lowLevelOutputProcessor,
                                      AvatarEstimatorThread estimatorThread,
                                      AvatarControllerThread controllerThread,
                                      AvatarStepGeneratorThread stepGeneratorThread,
                                      AvatarAffinityInterface affinity,
                                      double schedulerDt,
                                      MonotonicTime period,
                                      TimestampProvider monotonicTimeProvider,
                                      boolean useRealtimeThreads,
                                      boolean useMultiThreading,
                                      YoVariableServer yoVariableServer,
                                      YoRegistry parentRegistry)
   {
      this.masterContext = masterContext;
      this.hardwareCommunicationInterface = hardwareCommunicationInterface;
      this.lowLevelOutputProcessor = lowLevelOutputProcessor;
      this.estimatorThread = estimatorThread;
      this.affinity = affinity;
      this.schedulerDt = schedulerDt;
      this.monotonicTimeProvider = monotonicTimeProvider;
      this.useRealtimeThreads = useRealtimeThreads;
      this.useMultiThreading = useMultiThreading;
      this.yoVariableServer = yoVariableServer;

      // Set up the different tasks
      List<HumanoidRobotControlTask> tasks = new ArrayList<>();

      if (controllerThread != null)
         tasks.add(setupControllerTaskAndThread(robotModel, controllerThread, masterFullRobotModel, yoVariableServer));

      if (stepGeneratorThread != null)
         tasks.add(setupStepGeneratorTaskAndThread(robotModel, stepGeneratorThread, masterFullRobotModel, yoVariableServer));

      // Set up the thread manager
      if (useMultiThreading)
         threadManager = new BarrierScheduledRobotController(prefix + "ThreadManager",
                                                             tasks,
                                                             masterContext,
                                                             BarrierScheduler.TaskOverrunBehavior.BUSY_WAIT,
                                                             schedulerDt);
      else
         threadManager = new SingleThreadedRobotController<>(prefix + "ThreadManager", tasks, masterContext);

      threadManager.initialize();

      periodicParameters = new PeriodicParameters(period);

      // Set up the master thread
      if (useRealtimeThreads)
      {
         masterThread = new RealtimeThread(affinity.getSchedulerPriority(), periodicParameters, this::doControl, prefix + "-master-thread");
         masterThread.setAffinity(affinity.getSchedulerThreadProcessor());
         threadScheduler = null;
      }
      else
      {
         threadScheduler = new PeriodicNonRealtimeThreadScheduler("ThreadingManager");
         masterThread = null;
      }

      registry.addChild(threadManager.getYoRegistry());
      parentRegistry.addChild(registry);
   }

   private HumanoidRobotControlTask setupControllerTaskAndThread(DRCRobotModel robotModel,
                                                                 AvatarControllerThread controllerThread,
                                                                 FullHumanoidRobotModel masterFullRobotModel,
                                                                 YoVariableServer yoVariableServer)
   {
      // Set up Controller Task
      int controllerDivisor = (int) Math.round(robotModel.getControllerDT() / schedulerDt);
      if (!Precision.equals(robotModel.getControllerDT() / schedulerDt, controllerDivisor))
         throw new RuntimeException("Controller DT must be multiple of estimator DT.");

      controllerTask = new ControllerTask("Controller", controllerThread, controllerDivisor, schedulerDt, masterFullRobotModel);

      if (yoVariableServer != null)
         controllerTask.addCallbackPostTask(() -> yoVariableServer.update(controllerThread.getHumanoidRobotContextData().getTimestamp(),
                                                                          controllerThread.getYoVariableRegistry()));

      // Set up Controller Thread
      if (useRealtimeThreads && useMultiThreading)
      {
         RealtimeThread controllerRealtimeThread = new RealtimeThread(affinity.getControllerPriority(),
                                                                      controllerTask,
                                                                      controllerTask.getClass().getSimpleName() + "Thread");
         controllerRealtimeThread.setAffinity(affinity.getControlThreadProcessor());
         controllerRealtimeThread.start();
      }
      else if (!useRealtimeThreads && useMultiThreading)
      {
         Thread controllerNonRealtimeThread = new Thread(controllerTask, controllerTask.getClass().getSimpleName() + "Thread");
         controllerNonRealtimeThread.start();
      }

      return controllerTask;
   }

   private HumanoidRobotControlTask setupStepGeneratorTaskAndThread(DRCRobotModel robotModel,
                                                                    AvatarStepGeneratorThread stepGeneratorThread,
                                                                    FullHumanoidRobotModel masterFullRobotModel,
                                                                    YoVariableServer yoVariableServer)
   {
      // Set up Step Generator Task
      int stepGeneratorDivisor = (int) Math.round(robotModel.getStepGeneratorDT() / schedulerDt);
      if (!Precision.equals(robotModel.getStepGeneratorDT() / schedulerDt, stepGeneratorDivisor))
         throw new RuntimeException("Step generator DT must be multiple of estimator DT.");

      stepGeneratorTask = new StepGeneratorTask("StepGenerator", stepGeneratorThread, stepGeneratorDivisor, schedulerDt, masterFullRobotModel);

      if (yoVariableServer != null)
         stepGeneratorTask.addCallbackPostTask(() -> yoVariableServer.update(stepGeneratorThread.getHumanoidRobotContextData().getTimestamp(),
                                                                             stepGeneratorThread.getYoVariableRegistry()));

      // Set up Step Generator Thread
      if (useRealtimeThreads && useMultiThreading)
      {
         RealtimeThread stepGeneratorRealtimeThread = new RealtimeThread(affinity.getStepGeneratorPriority(),
                                                                         stepGeneratorTask,
                                                                         stepGeneratorTask.getClass().getSimpleName() + "Thread");
         stepGeneratorRealtimeThread.setAffinity(affinity.getStepGeneratorThreadProcessor());
         stepGeneratorRealtimeThread.start();
      }
      else if (!useRealtimeThreads && useMultiThreading)
      {
         Thread stepGeneratorNonRealtimeThread = new Thread(stepGeneratorTask, stepGeneratorTask.getClass().getSimpleName() + "Thread");
         stepGeneratorNonRealtimeThread.start();
      }

      return stepGeneratorTask;
   }

   public void start()
   {
      if (useRealtimeThreads)
         masterThread.start();
      else
         threadScheduler.schedule(this::doControl, periodicParameters.getPeriod().asNanoseconds(), TimeUnit.NANOSECONDS);
   }

   public void join()
   {
      if (useRealtimeThreads)
         masterThread.join();
   }

   public void stop()
   {
      if (yoVariableServer != null)
         yoVariableServer.close();
      threadManager.dispose();
      if (!useRealtimeThreads)
      {
         threadScheduler.shutdown();
      }
   }

   private void doControl()
   {
      if (isPaused.getBooleanValue())
         return;

      // Copy measured robot data to master context
      hardwareCommunicationInterface.read(masterContext.getSensorDataContext());

      if (!hardwareCommunicationInterface.hasReceivedFirstState())
         return;

      masterContext.setTimestamp(monotonicTimeProvider.getTimestamp()); //TODO should this be before the hasReceivedFirstState check?

      // Update estimator thread
      estimatorThread.run();
      updateYoVariableServer();

      // Update all scheduler thread runnables
      for (int i = 0; i < estimatorSchedulerThreadRunnables.size(); i++)
         estimatorSchedulerThreadRunnables.get(i).run();

      // Run the barrier scheduler controller
      long barrierSchedulerStartTime = System.nanoTime();
      threadManager.doControl();
      barrierSchedulerTime.set(System.nanoTime() - barrierSchedulerStartTime);

      // Write desired commands to robot
      lowLevelOutputProcessor.update(masterContext.getJointDesiredOutputList());
      hardwareCommunicationInterface.write(lowLevelOutputProcessor.getProcessedDesiredOutput());
   }

   public void addRunnableOnControllerSchedulerThread(Runnable runnable)
   {
      controllerTask.addRunnableOnSchedulerThread(runnable);
   }

   public void addRunnableOnStepGeneratorSchedulerThread(Runnable runnable)
   {
      stepGeneratorTask.addRunnableOnSchedulerThread(runnable);
   }

   public void addRunnableOnEstimatorSchedulerThread(Runnable runnable)
   {
      estimatorSchedulerThreadRunnables.add(runnable);
   }

   public void pause()
   {
      isPaused.set(true);
   }

   public void resume()
   {
      isPaused.set(false);
   }

   private void updateYoVariableServer()
   {
      if (yoVariableServer == null)
         return;

      long timestamp = estimatorThread.getHumanoidRobotContextData().getTimestamp();
      if (timestamp != Long.MIN_VALUE)
      {
         yoVariableServer.update(timestamp, estimatorThread.getYoRegistry());
      }
   }
}
