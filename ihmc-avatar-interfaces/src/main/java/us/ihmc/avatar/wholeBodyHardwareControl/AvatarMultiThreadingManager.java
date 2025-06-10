package us.ihmc.avatar.wholeBodyHardwareControl;

import org.apache.commons.math3.util.Precision;
import us.ihmc.avatar.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.BarrierScheduledRobotController;
import us.ihmc.avatar.factory.DisposableRobotController;
import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.avatar.factory.SingleThreadedRobotController;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.FrequencyCalculator;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler;
import us.ihmc.log.LogTools;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.tools.TimestampProvider;
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
   private final YoRegistry rootRegistry;
   private final YoVariableServer yoVariableServer;

   private final YoBoolean isPaused = new YoBoolean("isPaused", registry);

   private final double schedulerDt;

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

   private final DisposableRobotController threadScheduler;

   private final PeriodicParameters periodicParameters;

   private final Runnable masterThread;
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
   private boolean running = false;

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
                                      YoRegistry rootRegistry)
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
      this.rootRegistry = rootRegistry;
      this.yoVariableServer = yoVariableServer;

      // Set up the different tasks
      List<HumanoidRobotControlTask> tasks = new ArrayList<>();

      if (controllerThread != null)
         tasks.add(setupControllerTaskAndThread(robotModel, controllerThread, masterFullRobotModel, yoVariableServer));

      if (stepGeneratorThread != null)
         tasks.add(setupStepGeneratorTaskAndThread(robotModel, stepGeneratorThread, masterFullRobotModel, yoVariableServer));

      // Set up the thread manager
      if (useMultiThreading)
         threadScheduler = new BarrierScheduledRobotController(prefix + "ThreadManager",
                                                               tasks,
                                                               masterContext,
                                                               BarrierScheduler.TaskOverrunBehavior.BUSY_WAIT,
                                                               schedulerDt);
      else
         threadScheduler = new SingleThreadedRobotController<>(prefix + "ThreadManager", tasks, masterContext);

      threadScheduler.initialize();

      periodicParameters = new PeriodicParameters(period);

      // Set up the master thread
      if (useRealtimeThreads)
      {
         masterThread = new RealtimeThread(affinity.getSchedulerPriority(), periodicParameters, this::runRealtime, prefix + "-master-thread");
         ((RealtimeThread) masterThread).setAffinity(affinity.getSchedulerThreadProcessor());
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
      int controllerDivisor = (int) Math.round(robotModel.getControllerDT() / schedulerDt);
      if (!Precision.equals(robotModel.getControllerDT() / schedulerDt, controllerDivisor))
         throw new RuntimeException("Controller DT must be multiple of scheduler DT.");

      controllerTask = new ControllerTask("Controller", controllerThread, controllerDivisor, schedulerDt, masterFullRobotModel);

      if (yoVariableServer != null)
         controllerTask.addCallbackPostTask(() -> yoVariableServer.update(controllerThread.getHumanoidRobotContextData().getTimestamp(),
                                                                          controllerThread.getYoVariableRegistry()));

      controllerTask.addCallbackPostTask(()->
                                         {
                                            controllerThreadFrequencyCalculator.ping();
                                            controllerThreadUpdateRate.set(controllerThreadFrequencyCalculator.getFrequency());
                                         });

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
         throw new RuntimeException("Step generator DT must be multiple of scheduler DT.");

      stepGeneratorTask = new StepGeneratorTask("StepGenerator", stepGeneratorThread, stepGeneratorDivisor, schedulerDt, masterFullRobotModel);

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
      {
         running = true;
         ((RealtimeThread) masterThread).start();
      }
      else
      {
         ((RepeatingTaskThread) masterThread).setFrequencyLimit(Conversions.secondsToHertz(schedulerDt));
         ((RepeatingTaskThread) masterThread).startRepeating();
      }
   }

   public void join()
   {
      if (useRealtimeThreads)
         ((RealtimeThread) masterThread).join();
   }

   public void stop()
   {
      hardwareCommunicationInterface.stop();
   }

   public void destroy()
   {
      hardwareCommunicationInterface.destroy();

      ThreadTools.sleep(500L);

      threadScheduler.dispose();

      if (yoVariableServer != null)
         yoVariableServer.close();

      running = false;

      if (useRealtimeThreads)
      {
         ((RealtimeThread) masterThread).join();
      }
      else
         ((RepeatingTaskThread) masterThread).stopRepeating();

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

      // Calculate update rate of master thread
      masterThreadFrequencyCalculator.ping();
      masterThreadUpdateRate.set(masterThreadFrequencyCalculator.getFrequency());

      // Copy measured robot data to master context
      hardwareCommunicationInterface.read(masterContext.getSensorDataContext());

      if (hardwareCommunicationInterface.hasReceivedFirstState())
      {
         masterContext.setTimestamp(monotonicTimeProvider.getTimestamp());

         // Update estimator thread
         long estimatorThreadStartTime = System.nanoTime();
         estimatorThread.run();
         estimatorThreadComputeTime.set(System.nanoTime() - estimatorThreadStartTime);
         estimatorThreadFrequencyCalculator.ping();
         estimatorThreadUpdateRate.set(estimatorThreadFrequencyCalculator.getFrequency());

         // Update all scheduler thread runnables
         for (int i = 0; i < estimatorSchedulerThreadRunnables.size(); i++)
            estimatorSchedulerThreadRunnables.get(i).run();

         // Run the thread scheduler
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

   public void addRunnableOnControllerThread(Runnable runnable)
   {
      controllerTask.addRunnableOnSchedulerThread(runnable);
   }

   public void addRunnableOnStepGeneratorThread(Runnable runnable)
   {
      stepGeneratorTask.addRunnableOnSchedulerThread(runnable);
   }

   public void addRunnableOnEstimatorThread(Runnable runnable)
   {
      estimatorSchedulerThreadRunnables.add(runnable);
   }

   private void updateYoVariableServer()
   {
      if (yoVariableServer == null)
         return;

      yoVariableServer.update(monotonicTimeProvider.getTimestamp(), rootRegistry);
   }
}
