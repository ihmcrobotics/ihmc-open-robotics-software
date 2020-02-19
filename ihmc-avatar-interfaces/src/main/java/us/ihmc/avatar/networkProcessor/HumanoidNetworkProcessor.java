package us.ihmc.avatar.networkProcessor;

import java.net.URI;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.FootstepPlanPostProcessingToolboxModule;
import us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule.FootstepPlanningToolboxModule;
import us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule.KinematicsPlanningToolboxModule;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxMessageLogger;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.modules.RosModule;
import us.ihmc.avatar.networkProcessor.modules.ZeroPoseMockRobotConfigurationDataPublisherModule;
import us.ihmc.avatar.networkProcessor.modules.mocap.IHMCMOCAPLocalizationModule;
import us.ihmc.avatar.networkProcessor.modules.mocap.MocapPlanarRegionsListManager;
import us.ihmc.avatar.networkProcessor.quadTreeHeightMap.HeightQuadTreeToolboxModule;
import us.ihmc.avatar.networkProcessor.reaStateUpdater.HumanoidAvatarREAStateUpdater;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionPublisher;
import us.ihmc.avatar.networkProcessor.walkingPreview.WalkingControllerPreviewToolboxModule;
import us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule.WholeBodyTrajectoryToolboxModule;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.log.LogTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotBehaviors.watson.TextToSpeechNetworkModule;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.tools.processManagement.JavaProcessSpawner;
import us.ihmc.tools.thread.CloseableAndDisposable;

public class HumanoidNetworkProcessor implements CloseableAndDisposable
{
   private static final String NETWORK_PROCESSOR_ROS2_NODE_NAME = "network_processor";
   private static final String DEFAULT_REA_CONFIG_FILE_PATH = System.getProperty("user.home") + "/.ihmc/Configurations/defaultREAModuleConfiguration.txt";

   private final List<Runnable> modulesToStart = new ArrayList<>();
   private final List<CloseableAndDisposable> modulesToClose = new ArrayList<>();

   private final DRCRobotModel robotModel;
   private final PubSubImplementation pubSubImplementation;

   private Ros2Node ros2Node;
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
      if (parameters.isUseKinematicsPlanningToolboxModule())
         humanoidNetworkProcessor.setupKinematicsPlanningToolboxModule(parameters.isVisualizeKinematicsPlanningToolboxModule());
      if (parameters.isUseKinematicsStreamingToolboxModule())
         humanoidNetworkProcessor.setupKinematicsStreamingToolboxModule(null, null, parameters.isUseKinematicsStreamingToolboxModule());
      if (parameters.isUseFootstepPlanningToolboxModule())
         humanoidNetworkProcessor.setupFootstepPlanningToolboxModule(parameters.isVisualizeFootstepPlanningToolboxModule());
      if (parameters.isUseFootstepPostProcessingToolboxModule())
         humanoidNetworkProcessor.setupFootstepPostProcessingToolboxModule(parameters.isVisualizeFootstepPostProcessingToolboxModule());
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
      if (parameters.isUseRobotEnvironmentAwerenessModule())
         humanoidNetworkProcessor.setupRobotEnvironmentAwerenessModule(parameters.getREAConfigurationFilePath());
      if (parameters.isUseBipedalSupportPlanarRegionPublisherModule())
         humanoidNetworkProcessor.setupBipedalSupportPlanarRegionPublisherModule();
      if (parameters.isUseWalkingPreviewModule())
         humanoidNetworkProcessor.setupWalkingPreviewModule(parameters.isVisualizeWalkingPreviewModule());
      if (parameters.isUseHumanoidAvatarREAStateUpdater())
         humanoidNetworkProcessor.setupHumanoidAvatarREAStateUpdater();

      return humanoidNetworkProcessor;
   }

   public HumanoidNetworkProcessor(DRCRobotModel robotModel, PubSubImplementation pubSubImplementation)
   {
      this.robotModel = robotModel;
      this.pubSubImplementation = pubSubImplementation;
   }

   public void setupShutdownHook()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         LogTools.info("Shutting down network processor modules.");
         closeAndDispose();
         ThreadTools.sleep(10);
      }));
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

   public Ros2Node getRos2Node()
   {
      if (ros2Node == null)
      {
         ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, NETWORK_PROCESSOR_ROS2_NODE_NAME);
         modulesToClose.add(ros2Node::destroy);
      }
      return ros2Node;
   }

   public URI getRosURI()
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

   public FootstepPlanningToolboxModule setupFootstepPlanningToolboxModule(boolean enableYoVariableServer)
   {
      try
      {
         FootstepPlanningToolboxModule module = new FootstepPlanningToolboxModule(robotModel, null, enableYoVariableServer, pubSubImplementation);
         modulesToClose.add(module);
         return module;
      }
      catch (Throwable e)
      {
         reportFailure(e);
         return null;
      }
   }

   public FootstepPlanPostProcessingToolboxModule setupFootstepPostProcessingToolboxModule(boolean enableYoVariableServer)
   {
      try
      {
         FootstepPlanPostProcessingToolboxModule module = new FootstepPlanPostProcessingToolboxModule(robotModel,
                                                                                                      null,
                                                                                                      enableYoVariableServer,
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

   public IHMCMOCAPLocalizationModule setupMocapModule()
   {
      try
      {
         MocapPlanarRegionsListManager planarRegionsListManager = new MocapPlanarRegionsListManager();

         ROS2Tools.createCallbackSubscription(getRos2Node(),
                                              PlanarRegionsListMessage.class,
                                              REACommunicationProperties.publisherTopicNameGenerator,
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
      try
      {
         RosModule rosModule = new RosModule(robotModel, getRosURI(), simulatedSensorCommunicator, pubSubImplementation);
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
      try
      {
         DRCSensorSuiteManager sensorSuiteManager = robotModel.getSensorSuiteManager();
         if (robotModel.getTarget() == RobotTarget.SCS)
         {
            sensorSuiteManager.initializeSimulatedSensors(simulatedSensorCommunicator);
         }
         else
         {
            sensorSuiteManager.initializePhysicalSensors(getRosURI());
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

   public LIDARBasedREAModule setupRobotEnvironmentAwerenessModule(String reaConfigurationFilePath)
   {
      try
      {
         LIDARBasedREAModule module;
         if (reaConfigurationFilePath == null)
            module = LIDARBasedREAModule.createRemoteModule(DEFAULT_REA_CONFIG_FILE_PATH);
         else
            module = LIDARBasedREAModule.createRemoteModule(reaConfigurationFilePath);
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

   public HumanoidAvatarREAStateUpdater setupHumanoidAvatarREAStateUpdater()
   {
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

   private static void reportFailure(Throwable e)
   {
      LogTools.error("Failed to start a module in the network processor, stack trace:");
      e.printStackTrace();
   }

   public void start()
   {
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
