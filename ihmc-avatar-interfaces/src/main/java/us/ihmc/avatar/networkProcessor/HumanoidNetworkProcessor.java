package us.ihmc.avatar.networkProcessor;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule.KinematicsPlanningToolboxModule;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxMessageLogger;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.modules.RosModule;
import us.ihmc.avatar.networkProcessor.modules.ZeroPoseMockRobotConfigurationDataPublisherModule;
import us.ihmc.avatar.networkProcessor.modules.mocap.IHMCMOCAPLocalizationModule;
import us.ihmc.avatar.networkProcessor.modules.mocap.MocapPlanarRegionsListManager;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.quadTreeHeightMap.HeightQuadTreeToolboxModule;
import us.ihmc.avatar.networkProcessor.reaStateUpdater.HumanoidAvatarREAStateUpdater;
import us.ihmc.avatar.networkProcessor.reaStateUpdater.HumanoidAvatarStereoREAStateUpdater;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionPublisher;
import us.ihmc.avatar.networkProcessor.walkingPreview.WalkingControllerPreviewToolboxModule;
import us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule.WholeBodyTrajectoryToolboxModule;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.log.LogTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotBehaviors.watson.TextToSpeechNetworkModule;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.tools.processManagement.JavaProcessSpawner;
import us.ihmc.tools.thread.CloseableAndDisposable;

import java.net.URI;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.*;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.depthOutputTopic;

public class HumanoidNetworkProcessor implements CloseableAndDisposable
{
   private static final String NETWORK_PROCESSOR_ROS2_NODE_NAME = "network_processor";
   private static final String DEFAULT_REA_CONFIG_FILE_PATH = System.getProperty("user.home") + "/.ihmc/Configurations/defaultREAModuleConfiguration.txt";

   private boolean hasStarted = false;
   private boolean isShutdownHookSetup = false;
   private final List<Runnable> modulesToStart = new ArrayList<>();
   private final List<CloseableAndDisposable> modulesToClose = new ArrayList<>();

   private final DRCRobotModel robotModel;
   private final PubSubImplementation pubSubImplementation;

   private ROS2Node ros2Node;
   private URI rosURI;
   private ObjectCommunicator simulatedSensorCommunicator;

   public static HumanoidNetworkProcessor newFromParameters(DRCRobotModel robotModel, PubSubImplementation pubSubImplementation,
                                                            HumanoidNetworkProcessorParameters parameters)
   {
      HumanoidNetworkProcessor humanoidNetworkProcessor = new HumanoidNetworkProcessor(robotModel, pubSubImplementation);
      humanoidNetworkProcessor.setRosURI(parameters.getRosURI());
      humanoidNetworkProcessor.setSimulatedSensorCommunicator(parameters.getSimulatedSensorCommunicator());

      if (parameters.isUseTextToSpeechEngine())
         humanoidNetworkProcessor.setupTextToSpeechEngine();
      if (parameters.isUseZeroPoseRobotConfigurationPublisherModule())
         humanoidNetworkProcessor.setupZeroPoseRobotConfigurationPublisherModule();
      if (parameters.isUseWholeBodyTrajectoryToolboxModule())
         humanoidNetworkProcessor.setupWholeBodyTrajectoryToolboxModule(parameters.isVisualizeWholeBodyTrajectoryToolboxModule());
      if (parameters.isUseKinematicsToolboxModule())
         humanoidNetworkProcessor.setupKinematicsToolboxModule(parameters.isVisualizeKinematicsToolboxModule());
      if (parameters.isUseKinematicsPlanningToolboxModule())
         humanoidNetworkProcessor.setupKinematicsPlanningToolboxModule(parameters.isVisualizeKinematicsPlanningToolboxModule());
      if (parameters.isUseKinematicsStreamingToolboxModule())
         humanoidNetworkProcessor.setupKinematicsStreamingToolboxModule(null, null, parameters.isUseKinematicsStreamingToolboxModule());
      if (parameters.isUseFootstepPlanningToolboxModule())
         humanoidNetworkProcessor.setupFootstepPlanningToolboxModule();
      if (parameters.isUseMocapModule())
         humanoidNetworkProcessor.setupMocapModule();
      if (parameters.isUseBehaviorModule())
         humanoidNetworkProcessor.setupBehaviorModule(parameters.isVisualizeBehaviorModule(),
                                                      parameters.isUseAutomaticDiagnostic(),
                                                      parameters.getAutomatedDiagnosticInitialDelay());
      if (parameters.isUseROSModule())
         humanoidNetworkProcessor.setupRosModule();
      if (parameters.isUseSensorModule())
         humanoidNetworkProcessor.setupSensorModule();
      if (parameters.isUseHeightQuadTreeToolboxModule())
         humanoidNetworkProcessor.setupHeightQuadTreeToolboxModule();
      if (parameters.isUseFiducialDetectorToolboxModule())
         humanoidNetworkProcessor.setupFiducialDetectorToolboxModule();
      if (parameters.isUseObjectDetectorToolboxModule())
         humanoidNetworkProcessor.setupObjectDetectorToolboxModule();
      if (parameters.isUseRobotEnvironmentAwerenessModule())
         humanoidNetworkProcessor.setupRobotEnvironmentAwerenessModule(parameters.getREAConfigurationFilePath());
      if (parameters.isUseBipedalSupportPlanarRegionPublisherModule())
         humanoidNetworkProcessor.setupBipedalSupportPlanarRegionPublisherModule();
      if (parameters.isUseWalkingPreviewModule())
         humanoidNetworkProcessor.setupWalkingPreviewModule(parameters.isVisualizeWalkingPreviewModule());
      if (parameters.isUseHumanoidAvatarREAStateUpdater())
         humanoidNetworkProcessor.setupHumanoidAvatarLidarREAStateUpdater();

      return humanoidNetworkProcessor;
   }

   public HumanoidNetworkProcessor(DRCRobotModel robotModel, PubSubImplementation pubSubImplementation)
   {
      this.robotModel = robotModel;
      this.pubSubImplementation = pubSubImplementation;
   }

   public void setupShutdownHook()
   {
      if (isShutdownHookSetup)
      {
         LogTools.info("Shutdown hook already setup.");
         return;
      }

      isShutdownHookSetup = true;

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         LogTools.info("Shutting down network processor modules.");
         closeAndDispose();
         ThreadTools.sleep(10);
      }, getClass().getSimpleName() + "Shutdown"));
   }

   public void setRosURI(URI rosURI)
   {
      if (this.rosURI != null)
         throw new RuntimeException("The ROS URI has already been set or used, cannot modify it.");
      this.rosURI = rosURI;
   }

   public void setSimulatedSensorCommunicator(ObjectCommunicator simulatedSensorCommunicator)
   {
      if (this.simulatedSensorCommunicator != null)
         throw new RuntimeException("The simulated sensor communicator has already been set, cannot modify it.");
      this.simulatedSensorCommunicator = simulatedSensorCommunicator;
   }

   public ROS2Node getOrCreateROS2Node()
   {
      if (ros2Node == null)
      {
         LogTools.info("Creating ROS 2 node in {} mode", pubSubImplementation.name());
         ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, NETWORK_PROCESSOR_ROS2_NODE_NAME);
         modulesToClose.add(ros2Node::destroy);
      }
      return ros2Node;
   }

   public URI getOrCreateRosURI()
   {
      if (rosURI == null)
         rosURI = NetworkParameters.getROSURI();
      return rosURI;
   }

   public ObjectCommunicator getSimulatedSensorCommunicator()
   {
      if (simulatedSensorCommunicator == null)
         throw new RuntimeException("Simulated sensor communicator has not been set.");
      return simulatedSensorCommunicator;
   }

   public TextToSpeechNetworkModule setupTextToSpeechEngine()
   {
      checkIfModuleCanBeCreated(TextToSpeechNetworkModule.class);

      try
      {
         TextToSpeechNetworkModule module = new TextToSpeechNetworkModule(pubSubImplementation);
         modulesToClose.add(module);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public ZeroPoseMockRobotConfigurationDataPublisherModule setupZeroPoseRobotConfigurationPublisherModule()
   {
      checkIfModuleCanBeCreated(ZeroPoseMockRobotConfigurationDataPublisherModule.class);

      try
      {
         ZeroPoseMockRobotConfigurationDataPublisherModule module = new ZeroPoseMockRobotConfigurationDataPublisherModule(robotModel, pubSubImplementation);
         modulesToClose.add(module);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public WholeBodyTrajectoryToolboxModule setupWholeBodyTrajectoryToolboxModule(boolean enableYoVariableServer)
   {
      checkIfModuleCanBeCreated(WholeBodyTrajectoryToolboxModule.class);

      try
      {
         WholeBodyTrajectoryToolboxModule module = new WholeBodyTrajectoryToolboxModule(robotModel, enableYoVariableServer, pubSubImplementation);
         modulesToClose.add(module);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public KinematicsToolboxModule setupKinematicsToolboxModule(boolean enableYoVariableServer)
   {
      checkIfModuleCanBeCreated(KinematicsToolboxModule.class);

      try
      {
         KinematicsToolboxModule module = new KinematicsToolboxModule(robotModel, enableYoVariableServer, pubSubImplementation);
         modulesToClose.add(module);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public KinematicsPlanningToolboxModule setupKinematicsPlanningToolboxModule(boolean enableYoVariableServer)
   {
      checkIfModuleCanBeCreated(KinematicsPlanningToolboxModule.class);

      try
      {
         KinematicsPlanningToolboxModule module = new KinematicsPlanningToolboxModule(robotModel, enableYoVariableServer, pubSubImplementation);
         modulesToClose.add(module);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public KinematicsStreamingToolboxModule setupKinematicsStreamingToolboxModule(Class<?> launcherClass, String[] programArgs, boolean enableYoVariableServer)
   {
      try
      {
         modulesToClose.add(new KinematicsStreamingToolboxMessageLogger(robotModel.getSimpleRobotName(), pubSubImplementation));

         if (launcherClass == null)
         {
            checkIfModuleCanBeCreated(KinematicsStreamingToolboxModule.class);

            KinematicsStreamingToolboxModule module = new KinematicsStreamingToolboxModule(robotModel, enableYoVariableServer, pubSubImplementation);
            modulesToClose.add(module);
            return module;
         }
         else
         {
            Process process = new JavaProcessSpawner(true, true).spawn(launcherClass, programArgs);
            modulesToClose.add(process::destroy);
            return null;
         }
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public FootstepPlanningModule setupFootstepPlanningToolboxModule()
   {
	   
      checkIfModuleCanBeCreated(FootstepPlanningModule.class);

      try
      {
         FootstepPlanningModule module = FootstepPlanningModuleLauncher.createModule(robotModel, pubSubImplementation);
         modulesToClose.add(module);

         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public IHMCMOCAPLocalizationModule setupMocapModule()
   {
      checkIfModuleCanBeCreated(IHMCMOCAPLocalizationModule.class);

      try
      {
         MocapPlanarRegionsListManager planarRegionsListManager = new MocapPlanarRegionsListManager();

         ROS2Tools.createCallbackSubscriptionTypeNamed(getOrCreateROS2Node(),
                                                       PlanarRegionsListMessage.class,
                                                       REACommunicationProperties.outputTopic,
                                              s -> planarRegionsListManager.receivedPacket(s.takeNextData()));
         return new IHMCMOCAPLocalizationModule(robotModel, planarRegionsListManager);
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public IHMCHumanoidBehaviorManager setupBehaviorModule(boolean enableYoVariableServer, boolean automaticDiagnostic, double diagnosticInitialDelay)
   {
      checkIfModuleCanBeCreated(IHMCHumanoidBehaviorManager.class);

      try
      {
         HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
         LogModelProvider logModelProvider = robotModel.getLogModelProvider();
         IHMCHumanoidBehaviorManager behaviorManager;

         if (automaticDiagnostic)
         {
            behaviorManager = IHMCHumanoidBehaviorManager.createBehaviorModuleForAutomaticDiagnostic(robotModel.getSimpleRobotName(),
                                                                                                     robotModel.getFootstepPlannerParameters(),
                                                                                                     robotModel,
                                                                                                     robotModel,
                                                                                                     logModelProvider,
                                                                                                     enableYoVariableServer,
                                                                                                     sensorInformation,
                                                                                                     diagnosticInitialDelay);
         }
         else
         {
            behaviorManager = new IHMCHumanoidBehaviorManager(robotModel.getSimpleRobotName(),
                                                              robotModel.getFootstepPlannerParameters(),
                                                              robotModel,
                                                              robotModel,
                                                              logModelProvider,
                                                              enableYoVariableServer,
                                                              sensorInformation);
         }
         modulesToClose.add(behaviorManager);
         return behaviorManager;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public RosModule setupRosModule()
   {
      checkIfModuleCanBeCreated(RosModule.class);

      try
      {
         RosModule rosModule = new RosModule(robotModel, getOrCreateRosURI(), simulatedSensorCommunicator, pubSubImplementation);
         modulesToClose.add(rosModule);
         return rosModule;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public DRCSensorSuiteManager setupSensorModule()
   {
      LogTools.info("Setting up sensor module...");
      try
      {
         DRCSensorSuiteManager sensorSuiteManager = robotModel.getSensorSuiteManager(getOrCreateROS2Node());

         checkIfModuleCanBeCreated(sensorSuiteManager.getClass());

         if (robotModel.getTarget() == RobotTarget.SCS)
         {
            sensorSuiteManager.initializeSimulatedSensors(simulatedSensorCommunicator);
         }
         else
         {
            sensorSuiteManager.initializePhysicalSensors(getOrCreateRosURI());
         }
         modulesToClose.add(sensorSuiteManager);
         modulesToStart.add(sensorSuiteManager::connect);
         return sensorSuiteManager;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public HeightQuadTreeToolboxModule setupHeightQuadTreeToolboxModule()
   {
      checkIfModuleCanBeCreated(HeightQuadTreeToolboxModule.class);

      try
      {
         HeightQuadTreeToolboxModule module = new HeightQuadTreeToolboxModule(robotModel.getSimpleRobotName(),
                                                                              robotModel.createFullRobotModel(),
                                                                              robotModel.getLogModelProvider(),
                                                                              pubSubImplementation);
         modulesToClose.add(module);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }
   
   

   public FiducialDetectorToolboxModule setupFiducialDetectorToolboxModule()
   {
      checkIfModuleCanBeCreated(FiducialDetectorToolboxModule.class);

      try
      {
         FiducialDetectorToolboxModule module = new FiducialDetectorToolboxModule(robotModel.getSimpleRobotName(),
                                                                              robotModel.getTarget(),
                                                                              robotModel.createFullRobotModel(),
                                                                              robotModel.getLogModelProvider(),
                                                                              pubSubImplementation);
         modulesToClose.add(module);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }
   
   public ObjectDetectorToolboxModule setupObjectDetectorToolboxModule()
   {
      checkIfModuleCanBeCreated(ObjectDetectorToolboxModule.class);

      try
      {
         ObjectDetectorToolboxModule module = new ObjectDetectorToolboxModule(robotModel.getSimpleRobotName(),
                                                                              robotModel.createFullRobotModel(),
                                                                              robotModel.getLogModelProvider(),
                                                                              pubSubImplementation);
         modulesToClose.add(module);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }
   
   
   public LIDARBasedREAModule setupRobotEnvironmentAwerenessModule(String reaConfigurationFilePath)
   {
      checkIfModuleCanBeCreated(LIDARBasedREAModule.class);

      try
      {
         LIDARBasedREAModule module;

         REANetworkProvider networkProvider = new REAPlanarRegionPublicNetworkProvider(outputTopic,
                                                                                       lidarOutputTopic,
                                                                                       stereoOutputTopic,
                                                                                       depthOutputTopic);
         FilePropertyHelper filePropertyHelper;
         if (reaConfigurationFilePath != null)
            filePropertyHelper = new FilePropertyHelper(reaConfigurationFilePath);
         else
            filePropertyHelper = new FilePropertyHelper(DEFAULT_REA_CONFIG_FILE_PATH);
         module = LIDARBasedREAModule.createRemoteModule(filePropertyHelper, networkProvider);
         modulesToClose.add(module::stop);
         modulesToStart.add(module::start);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public BipedalSupportPlanarRegionPublisher setupBipedalSupportPlanarRegionPublisherModule()
   {
      checkIfModuleCanBeCreated(BipedalSupportPlanarRegionPublisher.class);

      try
      {
         BipedalSupportPlanarRegionPublisher module = new BipedalSupportPlanarRegionPublisher(robotModel, pubSubImplementation);
         modulesToClose.add(module);
         modulesToStart.add(module::start);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public WalkingControllerPreviewToolboxModule setupWalkingPreviewModule(boolean enableYoVariableServer)
   {
      checkIfModuleCanBeCreated(WalkingControllerPreviewToolboxModule.class);

      try
      {
         WalkingControllerPreviewToolboxModule module = new WalkingControllerPreviewToolboxModule(robotModel, enableYoVariableServer, pubSubImplementation);
         modulesToClose.add(module);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public HumanoidAvatarREAStateUpdater setupHumanoidAvatarLidarREAStateUpdater()
   {
      checkIfModuleCanBeCreated(HumanoidAvatarREAStateUpdater.class);

      try
      {
         HumanoidAvatarREAStateUpdater module = new HumanoidAvatarREAStateUpdater(robotModel, pubSubImplementation);
         modulesToClose.add(module);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public HumanoidAvatarStereoREAStateUpdater setupHumanoidAvatarRealSenseREAStateUpdater()
   {
      checkIfModuleCanBeCreated(HumanoidAvatarStereoREAStateUpdater.class);

      try
      {
         HumanoidAvatarStereoREAStateUpdater module = new HumanoidAvatarStereoREAStateUpdater(robotModel, pubSubImplementation, stereoInputTopic);
         modulesToClose.add(module);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   private static void reportFailure(Throwable e)
   {
      LogTools.error("Failed to start a module in the network processor, stack trace:");
      e.printStackTrace();
   }

   private void checkIfModuleCanBeCreated(Class<?> moduleType)
   {
      if (hasModuleBeenSetup(moduleType))
         throw new IllegalStateException("Attempting to instantiate a second time the module: " + moduleType.getSimpleName());
      if (hasStarted)
         throw new IllegalStateException("Attempting to instantiate a module but the network processor has already started.");
   }

   private boolean hasModuleBeenSetup(Class<?> moduleType)
   {
      return modulesToClose.stream().anyMatch(module -> module.getClass().equals(moduleType));
   }

   public void start()
   {
      hasStarted = true;

      for (Runnable module : modulesToStart)
      {
         try
         {
            module.run();
         }
         catch (Throwable e)
         {
            e.printStackTrace();
         }
      }
   }

   @Override
   public void closeAndDispose()
   {
      for (CloseableAndDisposable module : modulesToClose)
      {
         try
         {
            module.closeAndDispose();
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }

      modulesToClose.clear();
   }
}
