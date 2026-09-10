package us.ihmc.avatar.wholeBodyHardwareControl;

import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.avatar.AvatarEstimatorThreadFactory;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.logging.SCS2YoGraphicLogTools;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.log.LogTools;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * This class is responsible for creating the estimator threads along with the threading manager that handles the execution of the thread.
 *
 * @author Robert Griffin
 */
public class AvatarEstimatorProcessFactory
{
   private static final double GRAVITY = -9.81;
   private static final int ROS2_PRIORITY = 25;

   private final YoRegistry rootRegistry;

   // Robot model
   private final DRCRobotModel robotModel;

   // Hardware communication API
   private final EstimatorCommunicationInterface hardwareCommunicationInterface;

   // ROS stuff
   public final String IHMC_ROS_STATE_ESTIMATOR_NODE_NAME;
   private final AsyncROS2Node estimatorAsyncROS2Node;

   // The thread factories
   private final AvatarEstimatorThreadFactory estimatorThreadFactory;

   // The threads
   private final RequiredFactoryField<AvatarEstimatorThread> estimatorThread = new RequiredFactoryField<>("AvatarEstimatorThread");

   // Multi-threading manager
   private final RequiredFactoryField<AvatarEstimatorThreadManager> threadingManager = new RequiredFactoryField<>("AvatarMultiThreadingManager");

   // Output processor
   private final AvatarLowLevelOutputProcessor lowLevelOutputProcessor;

   // The remaining constructor arguments
   private final MonotonicTime period;
   private final double masterThreadDt;
   private final TimestampProvider monotonicTimeProvider;
   private final AvatarAffinityInterface affinity;
   private final boolean useRealtimeThreads;
   private final YoVariableServer yoVariableServer;

   public AvatarEstimatorProcessFactory(DRCRobotModel robotModel,
                                        FullHumanoidRobotModel fullRobotModel,
                                        EstimatorCommunicationInterface hardwareCommunicationInterface,
                                        SensorReaderFactory sensorReaderFactory,
                                        AvatarAffinityInterface affinity,
                                        boolean useRealtimeThreads,
                                        MonotonicTime period,
                                        double estimatorThreadDt,
                                        TimestampProvider monotonicTimeProvider,
                                        YoRegistry registry,
                                        YoVariableServer yoVariableServer)
   {
      this.robotModel = robotModel;
      this.period = period;
      this.masterThreadDt = estimatorThreadDt;
      this.monotonicTimeProvider = monotonicTimeProvider;
      this.hardwareCommunicationInterface = hardwareCommunicationInterface;
      this.affinity = affinity;
      this.useRealtimeThreads = useRealtimeThreads;
      this.rootRegistry = registry;
      this.yoVariableServer = yoVariableServer;

      // Estimator and controller ROS2 nodes
      IHMC_ROS_STATE_ESTIMATOR_NODE_NAME = robotModel.getSimpleRobotName().toLowerCase() + "_ihmc_state_estimator";

      PeriodicThreadSchedulerFactory ros2ThreadFactory;

      if (useRealtimeThreads)
         ros2ThreadFactory = new PeriodicRealtimeThreadSchedulerFactory(new PriorityParameters(ROS2_PRIORITY));
      else
         ros2ThreadFactory = new PeriodicNonRealtimeThreadSchedulerFactory();

      estimatorAsyncROS2Node = new AsyncROS2Node(IHMC_ROS_STATE_ESTIMATOR_NODE_NAME);

      // Set up low-level output processor
      lowLevelOutputProcessor = new AvatarLowLevelOutputProcessor(robotModel.getSimpleRobotName().toLowerCase(), fullRobotModel.getControllableOneDoFJoints(), estimatorThreadDt, registry);

      // Setup state estimator factory
      estimatorThreadFactory = createStateEstimatorFactory(robotModel, fullRobotModel, sensorReaderFactory);
   }

   public void start()
   {
      hardwareCommunicationInterface.start();
      threadingManager.get().start();
   }

   public void join()
   {
      threadingManager.get().join();
   }

   public void stop()
   {
      System.out.println("Calling stop in the multi-threading factory");
      hardwareCommunicationInterface.stop();
      threadingManager.get().stop();
   }

   public void destroy()
   {
      this.stop();
      System.out.println("Calling destroy in the estimator-threading factory");

      estimatorAsyncROS2Node.close();
      System.out.println("Estimator node has been destroyed");

      hardwareCommunicationInterface.destroy();
      System.out.println("Hardware communication node has been destroyed");

      threadingManager.get().destroy();
   }

   public AvatarEstimatorThreadManager buildThreadAndThreadingManager()
   {
      // Create estimator thread
      estimatorThread.set(estimatorThreadFactory.createAvatarEstimatorThread());

      // Add estimator thread registry as child to the root registry (since estimator thread is essentially our master thread)
      rootRegistry.addChild(estimatorThread.get().getYoRegistry());

      // Set the root registry as the YoVariableServer's main registry
      yoVariableServer.setMainRegistry(rootRegistry,
                                       estimatorThread.get().getFullRobotModel().getRootJoint().subtreeList(),
                                       SCS2YoGraphicLogTools.toYoGraphicsData(estimatorThread.get().getSCS2YoGraphics()));

      // Create threading manager
      threadingManager.set(new AvatarEstimatorThreadManager(robotModel.getSimpleRobotName().toLowerCase(),
                                                           hardwareCommunicationInterface,
                                                           lowLevelOutputProcessor,
                                                           estimatorThread.get(),
                                                           affinity,
                                                            masterThreadDt,
                                                           period,
                                                           monotonicTimeProvider,
                                                           useRealtimeThreads,
                                                           yoVariableServer,
                                                           rootRegistry));

      return threadingManager.get();
   }

   /**
    * Create Estimator Factory
    */
   private AvatarEstimatorThreadFactory createStateEstimatorFactory(DRCRobotModel robotModel, FullHumanoidRobotModel fullRobotModel, SensorReaderFactory sensorReaderFactory)
   {
      LogTools.info("The Squirrel estimates the number of acorns he needs for winter. Not many in Florida he thinks");
      HumanoidRobotContextDataFactory estimatorContextDataFactory = new HumanoidRobotContextDataFactory();

      AvatarEstimatorThreadFactory avatarEstimatorThreadFactory = new AvatarEstimatorThreadFactory();
      avatarEstimatorThreadFactory.setROS2Info(estimatorAsyncROS2Node, robotModel.getSimpleRobotName());

      avatarEstimatorThreadFactory.configureWithWholeBodyControllerParameters(robotModel);
      avatarEstimatorThreadFactory.setEstimatorFullRobotModel(fullRobotModel);
      avatarEstimatorThreadFactory.setSensorReaderFactory(sensorReaderFactory);
      avatarEstimatorThreadFactory.setHumanoidRobotContextDataFactory(estimatorContextDataFactory);
      avatarEstimatorThreadFactory.setGravity(GRAVITY);

      return avatarEstimatorThreadFactory;
   }

   public YoRegistry getEstimatorRegistry()
   {
      return estimatorThread.get().getYoRegistry();
   }

   public YoGraphicGroupDefinition getEstimatorYoGraphics()
   {
      return estimatorThread.get().getSCS2YoGraphics();
   }

   public AvatarEstimatorThreadManager getThreadingManager()
   {
      return threadingManager.get();
   }
}
