package us.ihmc.avatar.wholeBodyHardwareControl;

import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.FrequencyCalculator;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PeriodicParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is responsible for converting AvatarEstimatorThread either a regular thread or realtime thread. They are
 * then organized into either a SingleThreadedRobotController (in the case of single threading), or a
 * BarrierScheduledRobotController (in the case of multi-threading). These dictate the execution
 * of each thread and our entire control process itself. Lastly, this class manages the passing of
 * measured and desired robot data to and from a hardware communication interface.
 *
 * @author Stefan Fasano
 */
public class AvatarEstimatorThreadManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoRegistry rootRegistry;
   private final YoVariableServer yoVariableServer;

   private final YoBoolean isPaused = new YoBoolean("isPaused", registry);

   private final double estimatorThreadDt;

   private final YoDouble estimatorThreadUpdateRate = new YoDouble("estimatorThreadUpdateRate", registry);
   private final FrequencyCalculator estimatorThreadFrequencyCalculator = new FrequencyCalculator(false);
   private final YoLong estimatorThreadComputeTime = new YoLong("estimatorThreadComputeTime", registry);

   private final Runnable estimatorThreadRunnable;

   private final AvatarEstimatorThread estimatorThread;
   private final List<Runnable> preEstimatorThreadRunnables = new ArrayList<>();
   private final List<Runnable> postEstimatorThreadRunnables = new ArrayList<>();

   private final EstimatorCommunicationInterface hardwareCommunicationInterface;

   private final TimestampProvider monotonicTimeProvider;

   private final boolean useRealtimeThreads;

   private volatile boolean running = false;

   public AvatarEstimatorThreadManager(String prefix,
                                       EstimatorCommunicationInterface hardwareCommunicationInterface,
                                       AvatarLowLevelOutputProcessor lowLevelOutputProcessor,
                                       AvatarEstimatorThread estimatorThread,
                                       AvatarAffinityInterface affinity,
                                       double masterThreadDt,
                                       MonotonicTime period,
                                       TimestampProvider monotonicTimeProvider,
                                       boolean useRealtimeThreads,
                                       YoVariableServer yoVariableServer,
                                       YoRegistry rootRegistry)
   {
      this.hardwareCommunicationInterface = hardwareCommunicationInterface;
      this.estimatorThread = estimatorThread;
      this.estimatorThreadDt = masterThreadDt;
      this.monotonicTimeProvider = monotonicTimeProvider;
      this.useRealtimeThreads = useRealtimeThreads;
      this.rootRegistry = rootRegistry;
      this.yoVariableServer = yoVariableServer;

      // Set up the master thread
      if (useRealtimeThreads)
      {
         estimatorThreadRunnable = new RealtimeThread(affinity.getEstimatorThreadPriority(), new PeriodicParameters(period), this::runRealtime, prefix + "-master-thread");
         ((RealtimeThread) estimatorThreadRunnable).setAffinity(affinity.getEstimatorThreadProcessor());
      }
      else
      {
         estimatorThreadRunnable = new RepeatingTaskThread(prefix + "-master-thread", this::run, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      }

      // Add fault listener to unservo robot quickly in the event of a fault
      hardwareCommunicationInterface.addFaultListener(change ->
                                                      {
                                                         if (hardwareCommunicationInterface.hasRobotFaulted())
                                                            lowLevelOutputProcessor.unservoRobotQuickly();
                                                      });

      hardwareCommunicationInterface.addSoftEStopListener(change -> lowLevelOutputProcessor.unservoRobotQuickly());

      rootRegistry.addChild(registry);
   }

   public void start()
   {
      if (useRealtimeThreads)
      {
         running = true;
         ((RealtimeThread) estimatorThreadRunnable).start();
      }
      else
      {
         ((RepeatingTaskThread) estimatorThreadRunnable).setFrequencyLimit(Conversions.secondsToHertz(estimatorThreadDt));
         ((RepeatingTaskThread) estimatorThreadRunnable).startRepeating();
      }
   }

   public void join()
   {
      if (useRealtimeThreads)
         ((RealtimeThread) estimatorThreadRunnable).join();
      else
         ExceptionTools.handle(() -> ((RepeatingTaskThread) estimatorThreadRunnable).join(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public void stop()
   {
      hardwareCommunicationInterface.stop();
   }

   public void destroy()
   {
      hardwareCommunicationInterface.destroy();

      ThreadTools.sleep(500L);

      if (yoVariableServer != null)
         yoVariableServer.close();

      running = false;

      if (useRealtimeThreads)
      {
         ((RealtimeThread) estimatorThreadRunnable).join();
         ((RealtimeThread) estimatorThreadRunnable).finalize();
      }
      else
      {
         ((RepeatingTaskThread) estimatorThreadRunnable).stopRepeating();
      }

      ThreadTools.sleep(1000L);

      System.out.println("ESTIMATOR THREAD SHUTDOWN COMPLETE -- EXITING");
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
         long idleTime = ((RealtimeThread) estimatorThreadRunnable).waitForNextPeriod();
         if (idleTime > 0)
            run();
      }
   }

   private void run()
   {
      if (isPaused.getBooleanValue())
         return;

      // Calculate update rate of master thread
      estimatorThreadFrequencyCalculator.ping();
      estimatorThreadUpdateRate.set(estimatorThreadFrequencyCalculator.getFrequency());

      // Copy measured robot data to master context
      hardwareCommunicationInterface.read(estimatorThread.getHumanoidRobotContextData().getSensorDataContext());

      if (hardwareCommunicationInterface.hasReceivedFirstState())
      {
         estimatorThread.getHumanoidRobotContextData().setTimestamp(monotonicTimeProvider.getTimestamp());

         // Update all pre-estimator thread runnables
         for (int i = 0; i < preEstimatorThreadRunnables.size(); i++)
            preEstimatorThreadRunnables.get(i).run();

         // Update estimator thread
         long estimatorThreadStartTime = System.nanoTime();
         estimatorThread.run();
         estimatorThreadComputeTime.set(System.nanoTime() - estimatorThreadStartTime);

         // Update all post-estimator thread runnables
         for (int i = 0; i < postEstimatorThreadRunnables.size(); i++)
            postEstimatorThreadRunnables.get(i).run();

         // Write desired commands to robot
         hardwareCommunicationInterface.write(estimatorThread.getHumanoidRobotContextData().getProcessedJointData());
         // TODO this is where we need to publish the current state.
      }

      // Update YoVariable server
      updateYoVariableServer();
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
}
