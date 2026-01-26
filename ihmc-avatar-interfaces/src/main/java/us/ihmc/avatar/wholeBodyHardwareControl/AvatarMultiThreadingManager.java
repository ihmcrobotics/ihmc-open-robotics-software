package us.ihmc.avatar.wholeBodyHardwareControl;

import org.apache.commons.math3.util.Precision;
import us.ihmc.avatar.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.BarrierScheduledRobotController;
import us.ihmc.avatar.factory.DisposableRobotController;
import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.avatar.factory.SingleThreadedRobotController;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.IKStreamingRTPluginFactory;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.IKStreamingRTPluginFactory.IKStreamingRTThread;
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
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
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

   private final YoDouble estimatorThreadUpdateRate = new YoDouble("estimatorThreadUpdateRate", registry);
   private final FrequencyCalculator estimatorThreadFrequencyCalculator = new FrequencyCalculator(false);
   private final YoLong estimatorThreadComputeTime = new YoLong("estimatorThreadComputeTime", registry);

   private final DisposableRobotController threadScheduler;

   private final PeriodicParameters periodicParameters;

   private final Runnable masterThread;
   private final List<Runnable> childThreads;

   private final AvatarEstimatorThread estimatorThread;
   private final List<Runnable> preEstimatorThreadRunnables = new ArrayList<>();
   private final List<Runnable> postEstimatorThreadRunnables = new ArrayList<>();

   private final RealtimeROS2Node estimatorROS2Node;

   private final HumanoidRobotContextData masterContext;

   private final HardwareCommunicationInterface hardwareCommunicationInterface;

   private final TimestampProvider monotonicTimeProvider;

   private final boolean useRealtimeThreads;

   private final AvatarLowLevelOutputProcessor lowLevelOutputProcessor;
   private volatile boolean running = false;

   public AvatarMultiThreadingManager(String prefix,
                                      RealtimeROS2Node estimatorROS2Node,
                                      HumanoidRobotContextData masterContext,
                                      HardwareCommunicationInterface hardwareCommunicationInterface,
                                      AvatarLowLevelOutputProcessor lowLevelOutputProcessor,
                                      AvatarEstimatorThread estimatorThread,
                                      List<HumanoidRobotControlTask> tasks,
                                      List<Runnable> childThreads,
                                      AvatarAffinityInterface affinity,
                                      double masterThreadDt,
                                      MonotonicTime period,
                                      TimestampProvider monotonicTimeProvider,
                                      boolean useRealtimeThreads,
                                      boolean useMultiThreading,
                                      YoVariableServer yoVariableServer,
                                      YoRegistry rootRegistry)
   {
      this.estimatorROS2Node = estimatorROS2Node;
      this.masterContext = masterContext;
      this.hardwareCommunicationInterface = hardwareCommunicationInterface;
      this.lowLevelOutputProcessor = lowLevelOutputProcessor;
      this.estimatorThread = estimatorThread;
      this.childThreads = childThreads;
      this.masterThreadDt = masterThreadDt;
      this.monotonicTimeProvider = monotonicTimeProvider;
      this.useRealtimeThreads = useRealtimeThreads;
      this.rootRegistry = rootRegistry;
      this.yoVariableServer = yoVariableServer;

      // Set up the thread manager
      if (useMultiThreading)
         threadScheduler = new BarrierScheduledRobotController(prefix + "ThreadScheduler",
                                                               tasks,
                                                               masterContext,
                                                               BarrierScheduler.TaskOverrunBehavior.SKIP_SCHEDULER_TICK,
                                                               masterThreadDt);
      else
         threadScheduler = new SingleThreadedRobotController<>(prefix + "ThreadScheduler", tasks, masterContext);

      threadScheduler.initialize();

      periodicParameters = new PeriodicParameters(period);

      // Set up the master thread
      if (useRealtimeThreads)
      {
         masterThread = new RealtimeThread(affinity.getMasterThreadPriority(), periodicParameters, this::runRealtime, prefix + "-master-thread");
         ((RealtimeThread) masterThread).setAffinity(affinity.getMasterThreadProcessor());
      }
      else
      {
         masterThread = new RepeatingTaskThread(prefix + "-master-thread", this::run, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      }

      // Setup JVM statistics
      PeriodicThreadSchedulerFactory jvmSchedulerFactory;
      if (useRealtimeThreads)
         jvmSchedulerFactory = new PeriodicRealtimeThreadSchedulerFactory(new PriorityParameters(JVM_STATISTICS_PRIORITY));
      else
         jvmSchedulerFactory = new PeriodicNonRealtimeThreadSchedulerFactory();

      jvmStatisticsGenerator = new JVMStatisticsGenerator(yoVariableServer, jvmSchedulerFactory);
      jvmStatisticsGenerator.addVariablesToStatisticsGenerator(yoVariableServer);

      // Add fault listener to unservo robot quickly in the event of a fault
      hardwareCommunicationInterface.addFaultListener(change ->
                                                      {
                                                         if (hardwareCommunicationInterface.hasRobotFaulted())
                                                            lowLevelOutputProcessor.unservoRobotQuickly();
                                                      });

      hardwareCommunicationInterface.addSoftEStopListener(change -> lowLevelOutputProcessor.unservoRobotQuickly());

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
      estimatorROS2Node.spin();
      hardwareCommunicationInterface.start();
      jvmStatisticsGenerator.start();
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

   public void join()
   {
      if (useRealtimeThreads)
         ((RealtimeThread) masterThread).join();
      else
         ExceptionTools.handle(() -> ((RepeatingTaskThread) masterThread).join(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public void stop()
   {
      estimatorROS2Node.stopSpinning();
      hardwareCommunicationInterface.stop();
      jvmStatisticsGenerator.stop();
   }

   public void destroy()
   {
      estimatorROS2Node.destroy();
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
         for (int i = 0; i < childThreads.size(); i++)
            ((RepeatingTaskThread) childThreads.get(i)).stopRepeating();

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

   private void run()
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

      if (hardwareCommunicationInterface.hasReceivedFirstState())
      {
         masterContext.setTimestamp(monotonicTimeProvider.getTimestamp());

         // Update all pre-estimator thread runnables
         for (int i = 0; i < preEstimatorThreadRunnables.size(); i++)
            preEstimatorThreadRunnables.get(i).run();

         // Update estimator thread
         long estimatorThreadStartTime = System.nanoTime();
         estimatorThread.run();
         estimatorThreadComputeTime.set(System.nanoTime() - estimatorThreadStartTime);
         estimatorThreadFrequencyCalculator.ping();
         estimatorThreadUpdateRate.set(estimatorThreadFrequencyCalculator.getFrequency());

         // Update all post-estimator thread runnables
         for (int i = 0; i < postEstimatorThreadRunnables.size(); i++)
            postEstimatorThreadRunnables.get(i).run();

         // Run the thread scheduler. This will update the controller, step generator, and IK streaming threads (if they exist)
         long barrierSchedulerStartTime = System.nanoTime();
         threadScheduler.doControl();
         threadSchedulerComputeTime.set(System.nanoTime() - barrierSchedulerStartTime);

         // Write desired commands to robot
         lowLevelOutputProcessor.update(masterContext.getJointDesiredOutputList());
         hardwareCommunicationInterface.write(lowLevelOutputProcessor.getProcessedDesiredOutput(), lowLevelOutputProcessor.getMasterGain().getValue());
      }

      // Update YoVariable server
      updateYoVariableServer();
   }

   public YoRegistry getEstimatorRegistry()
   {
      return estimatorThread.getYoRegistry();
   }

   public YoGraphicGroupDefinition getEstimatorYoGraphics()
   {
      return estimatorThread.getSCS2YoGraphics();
   }

   public RealtimeROS2Node getEstimatorROS2Node()
   {
      return estimatorROS2Node;
   }

   public void addPreEstimatorThreadRunnable(Runnable runnable)
   {
      preEstimatorThreadRunnables.add(runnable);
   }

   public void addPostEstimatorThreadRunnable(Runnable runnable)
   {
      postEstimatorThreadRunnables.add(runnable);
   }

   private void updateYoVariableServer()
   {
      if (yoVariableServer == null)
         return;

      yoVariableServer.update(monotonicTimeProvider.getTimestamp(), rootRegistry);
   }

   public HardwareCommunicationInterface getHardwareCommunicationInterface()
   {
      return hardwareCommunicationInterface;
   }

   public AvatarLowLevelOutputProcessor getLowLevelOutputProcessor()
   {
      return lowLevelOutputProcessor;
   }
}
