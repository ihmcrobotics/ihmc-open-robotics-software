package us.ihmc.avatar.wholeBodyHardwareControl;

import us.ihmc.avatar.*;
import us.ihmc.avatar.factory.BarrierScheduledRobotController;
import us.ihmc.avatar.factory.DisposableRobotController;
import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.avatar.factory.SingleThreadedRobotController;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.FrequencyCalculator;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.util.JVMStatisticsGenerator;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;
import java.util.ArrayList;
import java.util.List;

/**
 * This class is responsible for converting AvatarEstimatorThread, AvatarControllerThread, and
 * AvatarStepGenerator thread into tasks and either regular threads or realtime threads. They are
 * then organized into either a SingleThreadedRobotController (in the case of single threading), or a
 * BarrierScheduledRobotController (in the case of multi-threading). These dictate the execution
 * of each thread and our entire control process itself. Lastly, this class manages the passing of
 * measured and desired robot data to and from a hardware communication interface.
 *
 * @author Stefan Fasano
 */
public class AvatarMultiThreadingManager
{
   private static final int JVM_STATISTICS_PRIORITY = 5;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final JVMStatisticsGenerator jvmStatisticsGenerator;
   private final YoRegistry rootRegistry;
   private final YoVariableServer yoVariableServer;

   private final YoBoolean isPaused = new YoBoolean("isPaused", registry);
   private final YoBoolean listenToBlockingCondition = new YoBoolean("listenToBlockingCondition", registry);
   private BooleanProvider blockingProvider;

   private final double masterThreadDt;

   private final YoLong threadSchedulerComputeTime = new YoLong("threadSchedulerComputeTime", registry);

   private final YoDouble masterThreadUpdateRate = new YoDouble("masterThreadUpdateRate", registry);
   private final FrequencyCalculator masterThreadFrequencyCalculator = new FrequencyCalculator(false);

   private final DisposableRobotController threadScheduler;

   private final Runnable masterThread;
   private final List<Runnable> childThreads;

   private final List<Runnable> preMasterThreadRunnables = new ArrayList<>();
   private final List<Runnable> postMasterThreadRunnables = new ArrayList<>();
   private final List<Runnable> masterThreadRunnables = new ArrayList<>();

   private final HumanoidRobotContextData masterContext;

   private final HardwareCommunicationInterface hardwareCommunicationInterface;

   private final TimestampProvider monotonicTimeProvider;

   private final boolean useRealtimeThreads;

   private final AvatarLowLevelOutputProcessor lowLevelOutputProcessor;
   private volatile boolean running = false;

   public AvatarMultiThreadingManager(String prefix,
                                      HumanoidRobotContextData masterContext,
                                      HardwareCommunicationInterface hardwareCommunicationInterface,
                                      AvatarLowLevelOutputProcessor lowLevelOutputProcessor,
                                      List<Runnable> childThreads,
                                      List<HumanoidRobotControlTask> childThreadTasks,
                                      AvatarAffinityInterface affinity,
                                      double masterThreadDt,
                                      MonotonicTime period,
                                      TimestampProvider monotonicTimeProvider,
                                      boolean useRealtimeThreads,
                                      boolean useMultiThreading,
                                      YoVariableServer yoVariableServer,
                                      YoRegistry rootRegistry)
   {
      this(prefix,
           masterContext,
           hardwareCommunicationInterface,
           lowLevelOutputProcessor,
           childThreads,
           childThreadTasks,
           affinity,
           masterThreadDt,
           period,
           monotonicTimeProvider,
           useRealtimeThreads,
           useMultiThreading,
           null,
           yoVariableServer,
           rootRegistry);
   }
   public AvatarMultiThreadingManager(String prefix,
                                      HumanoidRobotContextData masterContext,
                                      HardwareCommunicationInterface hardwareCommunicationInterface,
                                      AvatarLowLevelOutputProcessor lowLevelOutputProcessor,
                                      List<Runnable> childThreads,
                                      List<HumanoidRobotControlTask> childThreadTasks,
                                      AvatarAffinityInterface affinity,
                                      double masterThreadDt,
                                      MonotonicTime period,
                                      TimestampProvider monotonicTimeProvider,
                                      boolean useRealtimeThreads,
                                      boolean useMultiThreading,
                                      Runnable masterThread,
                                      YoVariableServer yoVariableServer,
                                      YoRegistry rootRegistry)
   {
      this.masterContext = masterContext;
      this.hardwareCommunicationInterface = hardwareCommunicationInterface;
      this.lowLevelOutputProcessor = lowLevelOutputProcessor;
      this.childThreads = childThreads;
      this.masterThreadDt = masterThreadDt;
      this.monotonicTimeProvider = monotonicTimeProvider;
      this.useRealtimeThreads = useRealtimeThreads;
      this.rootRegistry = rootRegistry;
      this.yoVariableServer = yoVariableServer;

      // Set up the thread manager
      if (useMultiThreading)
         threadScheduler = new BarrierScheduledRobotController(prefix + "ThreadScheduler",
                                                               childThreadTasks,
                                                               masterContext,
                                                               BarrierScheduler.TaskOverrunBehavior.SKIP_SCHEDULER_TICK,
                                                               masterThreadDt);
      else
         threadScheduler = new SingleThreadedRobotController<>(prefix + "ThreadScheduler", childThreadTasks, masterContext);

      threadScheduler.initialize();

      PeriodicParameters periodicParameters = new PeriodicParameters(period);

      // Set up the master thread
      if (masterThread == null)
      {
         if (useRealtimeThreads)
         {
            this.masterThread = new RealtimeThread(affinity.getMasterThreadPriority(), periodicParameters, this::runRealtime, prefix + "-master-thread");
            ((RealtimeThread) this.masterThread).setAffinity(affinity.getMasterThreadProcessor());
         }
         else
         {
            this.masterThread = new RepeatingTaskThread(prefix + "-master-thread", this::run, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         }
      }
      else
      {
         this.masterThread = masterThread;
      }

      // Setup JVM statistics
      PeriodicThreadSchedulerFactory jvmSchedulerFactory;
      if (useRealtimeThreads)
         jvmSchedulerFactory = new PeriodicRealtimeThreadSchedulerFactory(new PriorityParameters(JVM_STATISTICS_PRIORITY));
      else
         jvmSchedulerFactory = new PeriodicNonRealtimeThreadSchedulerFactory();

      jvmStatisticsGenerator = new JVMStatisticsGenerator(yoVariableServer, jvmSchedulerFactory);
      jvmStatisticsGenerator.addVariablesToStatisticsGenerator(yoVariableServer);

      registry.addChild(threadScheduler.getYoRegistry());
      rootRegistry.addChild(registry);
   }

   public void setListenToBlockingCondition(boolean listenToBlockingCondition)
   {
      this.listenToBlockingCondition.set(listenToBlockingCondition);
   }

   public void setBlockingProvider(BooleanProvider blockingProvider)
   {
      this.blockingProvider = blockingProvider;
   }

   public void start()
   {
      start(true);
   }

   public void start(boolean startMasterThread)
   {
      hardwareCommunicationInterface.start();
      jvmStatisticsGenerator.start();

      if (startMasterThread)
      {
         if (useRealtimeThreads)
         {
            running = true;
            ((RealtimeThread) masterThread).start();
         }
         else
         {
            ((RepeatingTaskThread) masterThread).setFrequencyLimit(Conversions.secondsToHertz(masterThreadDt));
            ((RepeatingTaskThread) masterThread).startRepeating();
         }
      }
   }

   public void join()
   {
      if (useRealtimeThreads)
         ((RealtimeThread) masterThread).join();
      else
         ExceptionTools.handle(() -> ((RepeatingTaskThread) masterThread).join(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public void stop()
   {
      hardwareCommunicationInterface.stop();
      jvmStatisticsGenerator.stop();
   }

   public void destroy()
   {
      System.out.println("Estimator node has been destroyed");

      hardwareCommunicationInterface.destroy();

      ThreadTools.sleep(500L);

      if (yoVariableServer != null)
         yoVariableServer.close();

      running = false;

      threadScheduler.dispose();
      if (useRealtimeThreads)
      {
         for (int i = 0; i < childThreads.size(); i++)
            ((RealtimeThread) childThreads.get(i)).finalize();

         ((RealtimeThread) masterThread).join();
         ((RealtimeThread) masterThread).finalize();
      }
      else
      {
         ((RepeatingTaskThread) masterThread).stopRepeating();
      }

      ThreadTools.sleep(1000L);

      System.out.println("MASTER THREAD SHUTDOWN COMPLETE -- EXITING");
   }

   public void pause()
   {
      isPaused.set(true);
   }

   public void resume()
   {
      isPaused.set(false);
   }

   private void runRealtime()
   {
      while(running)
      {
         long idleTime = ((RealtimeThread) masterThread).waitForNextPeriod();
         if (idleTime > 0)
            run();
      }
   }

   public void run()
   {
      if (isPaused.getBooleanValue())
         return;

      if (blockingProvider != null && listenToBlockingCondition.getBooleanValue() && blockingProvider.getValue())
      {
         // publish the older control state.
         hardwareCommunicationInterface.write(lowLevelOutputProcessor.getProcessedDesiredOutput(), lowLevelOutputProcessor.getMasterGain().getValue());
         return;
      }

      // Calculate update rate of master thread
      masterThreadFrequencyCalculator.ping();
      masterThreadUpdateRate.set(masterThreadFrequencyCalculator.getFrequency());

      // Copy measured robot data to master context
      hardwareCommunicationInterface.read(masterContext.getSensorDataContext());
      masterContext.setTimestamp(monotonicTimeProvider.getTimestamp());

      if (hardwareCommunicationInterface.hasReceivedFirstState())
      {
         // Update all pre-estimator thread runnables
         runAll(preMasterThreadRunnables);

         // Run the thread scheduler. This will update the controller, step generator, and IK streaming threads (if they exist)
         long barrierSchedulerStartTime = System.nanoTime();
         threadScheduler.doControl();
         threadSchedulerComputeTime.set(System.nanoTime() - barrierSchedulerStartTime);

         // Update all post-estimator thread runnables
         runAll(postMasterThreadRunnables);

         // Write desired commands to robot
         lowLevelOutputProcessor.update(masterContext.getJointDesiredOutputList());
         hardwareCommunicationInterface.write(lowLevelOutputProcessor.getProcessedDesiredOutput(), lowLevelOutputProcessor.getMasterGain().getValue());
      }

      // Update YoVariable server
      updateYoVariableServer();
   }

   private void updateYoVariableServer()
   {
      if (yoVariableServer == null)
         return;

      // Reuse masterContext's timestamp (set once per cycle, before doControl() dispatches the estimator
      // and controller) instead of re-reading the clock here. The estimator and controller copy that same
      // masterContext timestamp into their own context, so matching it here lets their independent
      // yoVariableServer.update() calls merge into this row instead of always writing a different ones.
      yoVariableServer.update(masterContext.getTimestamp(), rootRegistry);
   }

   public HardwareCommunicationInterface getHardwareCommunicationInterface()
   {
      return hardwareCommunicationInterface;
   }

   public AvatarLowLevelOutputProcessor getLowLevelOutputProcessor()
   {
      return lowLevelOutputProcessor;
   }

   public void addMasterThreadRunnable(Runnable runnable)
   {
      masterThreadRunnables.add(runnable);
   }

   public void addPreMasterThreadRunnable(Runnable runnable)
   {
      preMasterThreadRunnables.add(runnable);
   }

   public void addPostMasterThreadRunnable(Runnable runnable)
   {
      postMasterThreadRunnables.add(runnable);
   }

   static void runAll(List<Runnable> runnables)
   {
      for (int i = 0; i < runnables.size(); i ++)
         runnables.get(i).run();
   }
}
