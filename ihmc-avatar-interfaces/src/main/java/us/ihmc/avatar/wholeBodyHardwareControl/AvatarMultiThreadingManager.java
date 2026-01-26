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
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.util.JVMStatisticsGenerator;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.tools.TimestampProvider;
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

   private final YoDouble controllerThreadUpdateRate = new YoDouble("controllerThreadUpdateRate", registry);
   private final FrequencyCalculator controllerThreadFrequencyCalculator = new FrequencyCalculator(false);

   private final YoDouble stepGeneratorThreadUpdateRate = new YoDouble("stepGeneratorThreadUpdateRate", registry);
   private final FrequencyCalculator stepGeneratorThreadFrequencyCalculator = new FrequencyCalculator(false);

   private final YoDouble ikStreamingThreadUpdateRate = new YoDouble("ikStreamingThreadUpdateRate", registry);
   private final FrequencyCalculator ikStreamingThreadFrequencyCalculator = new FrequencyCalculator(false);

   private final DisposableRobotController threadScheduler;

   private final PeriodicParameters periodicParameters;

   private final Runnable masterThread;
   private final List<Runnable> childThreads = new ArrayList<>();

   private final AvatarEstimatorThread estimatorThread;
   private final List<Runnable> preEstimatorThreadRunnables = new ArrayList<>();
   private final List<Runnable> postEstimatorThreadRunnables = new ArrayList<>();

   private ControllerTask controllerTask;
   private StepGeneratorTask stepGeneratorTask;
   private final AvatarStepGeneratorThread stepGeneratorThread;

   private final AvatarAffinityInterface affinity;

   private final RealtimeROS2Node estimatorROS2Node;
   private final RealtimeROS2Node controllerROS2Node;

   private final HumanoidRobotContextData masterContext;

   private final HardwareCommunicationInterface hardwareCommunicationInterface;

   private final TimestampProvider monotonicTimeProvider;

   private final boolean useRealtimeThreads;
   private final boolean useMultiThreading;

   private final AvatarLowLevelOutputProcessor lowLevelOutputProcessor;
   private volatile boolean running = false;

   public AvatarMultiThreadingManager(String prefix,
                                      DRCRobotModel robotModel,
                                      RealtimeROS2Node estimatorROS2Node,
                                      RealtimeROS2Node controllerROS2Node,
                                      HumanoidRobotContextData masterContext,
                                      FullHumanoidRobotModel masterFullRobotModel,
                                      HardwareCommunicationInterface hardwareCommunicationInterface,
                                      AvatarLowLevelOutputProcessor lowLevelOutputProcessor,
                                      AvatarEstimatorThread estimatorThread,
                                      AvatarControllerThread controllerThread,
                                      AvatarStepGeneratorThread stepGeneratorThread,
                                      IKStreamingRTThread ikStreamingThread,
                                      AvatarAffinityInterface affinity,
                                      double masterThreadDt,
                                      MonotonicTime period,
                                      TimestampProvider monotonicTimeProvider,
                                      boolean useRealtimeThreads,
                                      boolean useMultiThreading,
                                      YoVariableServer yoVariableServer,
                                      JVMStatisticsGenerator jvmStatisticsGenerator,
                                      YoRegistry rootRegistry)
   {
      this.estimatorROS2Node = estimatorROS2Node;
      this.controllerROS2Node = controllerROS2Node;
      this.masterContext = masterContext;
      this.hardwareCommunicationInterface = hardwareCommunicationInterface;
      this.lowLevelOutputProcessor = lowLevelOutputProcessor;
      this.estimatorThread = estimatorThread;
      this.stepGeneratorThread = stepGeneratorThread;
      this.affinity = affinity;
      this.masterThreadDt = masterThreadDt;
      this.monotonicTimeProvider = monotonicTimeProvider;
      this.useRealtimeThreads = useRealtimeThreads;
      this.useMultiThreading = useMultiThreading;
      this.jvmStatisticsGenerator = jvmStatisticsGenerator;
      this.rootRegistry = rootRegistry;
      this.yoVariableServer = yoVariableServer;

      // Set up the different tasks
      List<HumanoidRobotControlTask> tasks = new ArrayList<>();

      if (controllerThread != null)
         tasks.add(setupControllerTaskAndThread(robotModel, controllerThread, masterFullRobotModel, yoVariableServer));

      if (stepGeneratorThread != null)
         tasks.add(setupStepGeneratorTaskAndThread(robotModel, stepGeneratorThread, masterFullRobotModel, yoVariableServer));

      if (ikStreamingThread != null)
         tasks.add(setupIKStreamingTaskAndThread(ikStreamingThread, yoVariableServer));

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

   private HumanoidRobotControlTask setupControllerTaskAndThread(DRCRobotModel robotModel,
                                                                 AvatarControllerThread controllerThread,
                                                                 FullHumanoidRobotModel masterFullRobotModel,
                                                                 YoVariableServer yoVariableServer)
   {
      // Set up Controller Task
      int controllerDivisor = (int) Math.round(robotModel.getControllerDT() / masterThreadDt);
      if (!Precision.equals(robotModel.getControllerDT() / masterThreadDt, controllerDivisor))
         throw new RuntimeException("Controller DT must be multiple of master thread DT.");

      controllerTask = new ControllerTask("Controller", controllerThread, controllerDivisor, masterThreadDt, masterFullRobotModel);

      if (yoVariableServer != null)
         controllerTask.addCallbackPostTask(() -> yoVariableServer.update(controllerThread.getHumanoidRobotContextData().getTimestamp(),
                                                                          controllerThread.getYoVariableRegistry()));

      controllerTask.addCallbackPostTask(lowLevelOutputProcessor::startDesiredsInterpolation);

      controllerTask.addCallbackPostTask(()->
                                         {
                                            controllerThreadFrequencyCalculator.ping();
                                            controllerThreadUpdateRate.set(controllerThreadFrequencyCalculator.getFrequency());
                                         });

      // Set up Controller Thread
      if (useRealtimeThreads && useMultiThreading)
      {
         RealtimeThread controllerRealtimeThread = new RealtimeThread(affinity.getControllerThreadPriority(),
                                                                      controllerTask,
                                                                      controllerTask.getClass().getSimpleName() + "Thread");
         controllerRealtimeThread.setAffinity(affinity.getControllerThreadProcessor());
         controllerRealtimeThread.start();

         childThreads.add(controllerRealtimeThread);
      }
      else if (!useRealtimeThreads && useMultiThreading)
      {
         Thread controllerNonRealtimeThread = new Thread(controllerTask, controllerTask.getClass().getSimpleName() + "Thread");
         controllerNonRealtimeThread.start();

         childThreads.add(controllerNonRealtimeThread);
      }

      return controllerTask;
   }

   private HumanoidRobotControlTask setupStepGeneratorTaskAndThread(DRCRobotModel robotModel,
                                                                    AvatarStepGeneratorThread stepGeneratorThread,
                                                                    FullHumanoidRobotModel masterFullRobotModel,
                                                                    YoVariableServer yoVariableServer)
   {
      // Set up Step Generator Task
      int stepGeneratorDivisor = (int) Math.round(robotModel.getStepGeneratorDT() / masterThreadDt);
      if (!Precision.equals(robotModel.getStepGeneratorDT() / masterThreadDt, stepGeneratorDivisor))
         throw new RuntimeException("Step generator DT must be multiple of master thread DT.");

      stepGeneratorTask = new StepGeneratorTask("StepGenerator", stepGeneratorThread, stepGeneratorDivisor, masterThreadDt, masterFullRobotModel);

      if (yoVariableServer != null)
         stepGeneratorTask.addCallbackPostTask(() -> yoVariableServer.update(stepGeneratorThread.getHumanoidRobotContextData().getTimestamp(),
                                                                             stepGeneratorThread.getYoVariableRegistry()));

      stepGeneratorTask.addCallbackPostTask(()->
                                         {
                                            stepGeneratorThreadFrequencyCalculator.ping();
                                            stepGeneratorThreadUpdateRate.set(stepGeneratorThreadFrequencyCalculator.getFrequency());
                                         });

      // Set up Step Generator Thread
      if (useRealtimeThreads && useMultiThreading)
      {
         RealtimeThread stepGeneratorRealtimeThread = new RealtimeThread(affinity.getStepGeneratorThreadPriority(),
                                                                         stepGeneratorTask,
                                                                         stepGeneratorTask.getClass().getSimpleName() + "Thread");
         stepGeneratorRealtimeThread.setAffinity(affinity.getStepGeneratorThreadProcessor());
         stepGeneratorRealtimeThread.start();

         childThreads.add(stepGeneratorRealtimeThread);
      }
      else if (!useRealtimeThreads && useMultiThreading)
      {
         Thread stepGeneratorNonRealtimeThread = new Thread(stepGeneratorTask, stepGeneratorTask.getClass().getSimpleName() + "Thread");
         stepGeneratorNonRealtimeThread.start();

         childThreads.add(stepGeneratorNonRealtimeThread);
      }

      return stepGeneratorTask;
   }

   private HumanoidRobotControlTask setupIKStreamingTaskAndThread(IKStreamingRTPluginFactory.IKStreamingRTThread ikStreamingThread,
                                                                  YoVariableServer yoVariableServer)
   {
      // Set up Step Generator Task
      IKStreamingRTPluginFactory.IKStreamingRTTask ikStreamingTask = IKStreamingRTPluginFactory.createIKStreamingRTTask(ikStreamingThread, masterThreadDt);

      if (yoVariableServer != null)
         ikStreamingTask.addCallbackPostTask(() -> yoVariableServer.update(ikStreamingThread.getHumanoidRobotContextData().getTimestamp(),
                                                                           ikStreamingThread.getYoVariableRegistry()));

      ikStreamingTask.addCallbackPostTask(() ->
                                          {
                                             ikStreamingThreadFrequencyCalculator.ping();
                                             ikStreamingThreadUpdateRate.set(ikStreamingThreadFrequencyCalculator.getFrequency());
                                          });

      // Set up Step Generator Thread
      if (useRealtimeThreads && useMultiThreading)
      {
         RealtimeThread ikStreamingRealtimeThread = new RealtimeThread(affinity.getIKStreamingThreadPriority(),
                                                                       ikStreamingTask,
                                                                       ikStreamingTask.getClass().getSimpleName() + "Thread");
         ikStreamingRealtimeThread.setAffinity(affinity.getIKStreamingThreadProcessor());
         ikStreamingRealtimeThread.start();

         childThreads.add(ikStreamingRealtimeThread);
      }
      else if (!useRealtimeThreads && useMultiThreading)
      {
         Thread ikStreamingNonRealtimeThread = new Thread(ikStreamingTask, ikStreamingTask.getClass().getSimpleName() + "Thread");
         ikStreamingNonRealtimeThread.start();

         childThreads.add(ikStreamingNonRealtimeThread);
      }

      return ikStreamingTask;
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
      controllerROS2Node.spin();
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
      controllerROS2Node.stopSpinning();
      hardwareCommunicationInterface.stop();
      jvmStatisticsGenerator.stop();
   }

   public void destroy()
   {
      estimatorROS2Node.destroy();
      System.out.println("Estimator node has been destroyed");

      controllerROS2Node.destroy();
      System.out.println("Controller node has been destroyed");

      hardwareCommunicationInterface.destroy();

      ThreadTools.sleep(500L);

      if (yoVariableServer != null)
         yoVariableServer.close();

      running = false;

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

      if (stepGeneratorThread != null)
         stepGeneratorThread.destroy();

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

   public RealtimeROS2Node getControllerROS2Node()
   {
      return controllerROS2Node;
   }

   public void addPreEstimatorThreadRunnable(Runnable runnable)
   {
      preEstimatorThreadRunnables.add(runnable);
   }

   public void addPostEstimatorThreadRunnable(Runnable runnable)
   {
      postEstimatorThreadRunnables.add(runnable);
   }

   public void addPreControllerThreadRunnable(Runnable runnable)
   {
      controllerTask.addCallbackPreTask(runnable);
   }

   public void addPostControllerThreadRunnable(Runnable runnable)
   {
      controllerTask.addCallbackPostTask(runnable);
   }

   public void addPreStepGeneratorThreadRunnable(Runnable runnable)
   {
      stepGeneratorTask.addCallbackPreTask(runnable);
   }

   public void addPostStepGeneratorThreadRunnable(Runnable runnable)
   {
      stepGeneratorTask.addCallbackPostTask(runnable);
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
