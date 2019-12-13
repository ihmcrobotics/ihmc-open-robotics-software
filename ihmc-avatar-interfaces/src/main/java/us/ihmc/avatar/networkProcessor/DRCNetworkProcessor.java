package us.ihmc.avatar.networkProcessor;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.FootstepPlanPostProcessingToolboxModule;
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
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
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

public class DRCNetworkProcessor implements CloseableAndDisposable
{
   private final boolean DEBUG = false;
   private final String[] programArgs;
   private final PubSubImplementation pubSubImplementation;
   private final List<CloseableAndDisposable> modules = new ArrayList<>();
   private final Ros2Node ros2Node;

   public DRCNetworkProcessor(DRCRobotModel robotModel, DRCNetworkModuleParameters params, PubSubImplementation pubSubImplementation)
   {
      this(null, robotModel, params, pubSubImplementation);
   }

   public DRCNetworkProcessor(String[] programArgs, DRCRobotModel robotModel, DRCNetworkModuleParameters params, PubSubImplementation pubSubImplementation)
   {
      this.programArgs = programArgs;
      this.pubSubImplementation = pubSubImplementation;
      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "network_processor");

      tryToStartModule(() -> setupRosModule(robotModel, params));
      tryToStartModule(() -> setupSensorModule(robotModel, params));
      tryToStartModule(() -> setupBehaviorModule(robotModel, params));
      tryToStartModule(() -> setupMocapModule(robotModel, params));
      tryToStartModule(() -> setupZeroPoseRobotConfigurationPublisherModule(robotModel, params));
      tryToStartModule(() -> setupWholebodyTrajectoryToolboxModule(robotModel, params));
      tryToStartModule(() -> setupKinematicsToolboxModule(robotModel, params));
      tryToStartModule(() -> setupKinematicsStreamingToolboxModule(robotModel, params));
      tryToStartModule(() -> setupKinematicsPlanningToolboxModule(robotModel, params));
      tryToStartModule(() -> setupFootstepPlanningToolboxModule(robotModel, params));
      tryToStartModule(() -> setupFootstepPostProcessingToolboxModule(robotModel, params));
      tryToStartModule(() -> addTextToSpeechEngine(params));
      tryToStartModule(() -> setupHeightQuadTreeToolboxModule(robotModel, params));
      tryToStartModule(() -> setupRobotEnvironmentAwerenessModule(params));
      tryToStartModule(() -> setupBipedalSupportPlanarRegionPublisherModule(robotModel, params));
      tryToStartModule(() -> setupWalkingPreviewModule(robotModel, params));
      tryToStartModule(() -> setupHumanoidAvatarREAStateUpdater(robotModel, params));

      LogTools.info("All modules in network processor are up and running!");

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         LogTools.info("Shutting down network processor modules.");
         closeAndDispose();
         ThreadTools.sleep(10);
      }));
   }

   private void addTextToSpeechEngine(DRCNetworkModuleParameters params)
   {
      if (params.isTextToSpeechModuleEnabled())
         modules.add(new TextToSpeechNetworkModule(pubSubImplementation));
   }

   private void setupZeroPoseRobotConfigurationPublisherModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      if (params.isZeroPoseRobotConfigurationPublisherEnabled())
         modules.add(new ZeroPoseMockRobotConfigurationDataPublisherModule(robotModel, pubSubImplementation));
   }

   private void setupWholebodyTrajectoryToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isWholeBodyTrajectoryToolboxEnabled())
         modules.add(new WholeBodyTrajectoryToolboxModule(robotModel, params.isWholeBodyTrajectoryToolboxVisualizerEnabled(), pubSubImplementation));
   }

   private void setupKinematicsToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isKinematicsToolboxEnabled())
         modules.add(new KinematicsToolboxModule(robotModel, params.isKinematicsToolboxVisualizerEnabled(), pubSubImplementation));
   }

   private void setupKinematicsPlanningToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isKinematicsPlanningToolboxEnabled())
         modules.add(new KinematicsPlanningToolboxModule(robotModel, false, pubSubImplementation));
   }

   private void setupKinematicsStreamingToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (!params.isKinematicsStreamingToolboxEnabled())
         return;
      if (params.getKinematicsStreamingToolboxLauncherClass() == null)
         modules.add(new KinematicsStreamingToolboxModule(robotModel, params.isKinematicsToolboxVisualizerEnabled(), pubSubImplementation));
      else
         new JavaProcessSpawner(true, true).spawn(params.getKinematicsStreamingToolboxLauncherClass(), programArgs);
      modules.add(new KinematicsStreamingToolboxMessageLogger(robotModel.getSimpleRobotName(), pubSubImplementation));
   }

   private void setupFootstepPlanningToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isFootstepPlanningToolboxEnabled())
         modules.add(new MultiStageFootstepPlanningModule(robotModel, null, params.isFootstepPlanningToolboxVisualizerEnabled(), pubSubImplementation));
   }

   private void setupFootstepPostProcessingToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (!params.isFootstepPostProcessingToolboxEnabled())
         return;

      modules.add(new FootstepPlanPostProcessingToolboxModule(robotModel,
                                                              null,
                                                              params.isFootstepPostProcessingToolboxVisualizerEnabled(),
                                                              pubSubImplementation));
   }

   private void setupMocapModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isMocapModuleEnabled())
      {
         MocapPlanarRegionsListManager planarRegionsListManager = new MocapPlanarRegionsListManager();

         ROS2Tools.createCallbackSubscription(ros2Node,
                                              PlanarRegionsListMessage.class,
                                              REACommunicationProperties.publisherTopicNameGenerator,
                                              s -> planarRegionsListManager.receivedPacket(s.takeNextData()));
         new IHMCMOCAPLocalizationModule(robotModel, planarRegionsListManager);

         String methodName = "setupMocapModule";
         printModuleConnectedDebugStatement(PacketDestination.MOCAP_MODULE, methodName);
      }
   }

   private void setupBehaviorModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isBehaviorModuleEnabled())
      {
         HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
         LogModelProvider logModelProvider = robotModel.getLogModelProvider();
         IHMCHumanoidBehaviorManager behaviorManager;

         if (params.isAutomaticDiagnosticEnabled())
         {
            behaviorManager = IHMCHumanoidBehaviorManager.createBehaviorModuleForAutomaticDiagnostic(robotModel.getSimpleRobotName(),
                                                                                                     robotModel.getFootstepPlannerParameters(),
                                                                                                     robotModel,
                                                                                                     robotModel,
                                                                                                     logModelProvider,
                                                                                                     params.isBehaviorVisualizerEnabled(),
                                                                                                     sensorInformation,
                                                                                                     params.getTimeToWaitBeforeStartingDiagnostics());
         }
         else
         {
            behaviorManager = new IHMCHumanoidBehaviorManager(robotModel.getSimpleRobotName(),
                                                              robotModel.getFootstepPlannerParameters(),
                                                              robotModel,
                                                              robotModel,
                                                              logModelProvider,
                                                              params.isBehaviorVisualizerEnabled(),
                                                              sensorInformation);
         }
         modules.add(behaviorManager);

         String methodName = "setupBehaviorModule ";
         printModuleConnectedDebugStatement(PacketDestination.BEHAVIOR_MODULE, methodName);
      }
   }

   private void setupRosModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isRosModuleEnabled())
         modules.add(new RosModule(robotModel, params.getRosUri(), params.getSimulatedSensorCommunicator(), pubSubImplementation));
   }

   private void setupSensorModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isSensorModuleEnabled())
      {
         DRCSensorSuiteManager sensorSuiteManager = robotModel.getSensorSuiteManager();
         if (params.isSimulatedSensorsEnabled())
         {
            sensorSuiteManager.initializeSimulatedSensors(params.getSimulatedSensorCommunicator());
         }
         else
         {
            sensorSuiteManager.initializePhysicalSensors(params.getRosUri());
         }
         sensorSuiteManager.connect();
         modules.add(sensorSuiteManager);
      }
   }

   // FIXME Do we need that still?
   //   private void setupROSAPIModule(DRCNetworkModuleParameters params) throws IOException
   //   {
   //      if (params.isROSAPICommunicatorEnabled())
   //      {
   //         PacketCommunicator rosAPICommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_API_COMMUNICATOR, NET_CLASS_LIST);
   //         packetRouter.attachPacketCommunicator(PacketDestination.ROS_API, rosAPICommunicator);
   //         rosAPICommunicator.connect();
   //         String methodName = "setupROSAPIModule ";
   //         printModuleConnectedDebugStatement(PacketDestination.ROS_API, methodName);
   //      }
   //   }

   private void setupHeightQuadTreeToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isHeightQuadTreeToolboxEnabled())
         modules.add(new HeightQuadTreeToolboxModule(robotModel.getSimpleRobotName(),
                                                     robotModel.createFullRobotModel(),
                                                     robotModel.getLogModelProvider(),
                                                     pubSubImplementation));
   }

   private void setupRobotEnvironmentAwerenessModule(DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isRobotEnvironmentAwerenessModuleEnabled())
      {
         try
         {
            LIDARBasedREAModule.createRemoteModule(System.getProperty("user.home") + "/.ihmc/Configurations/defaultREAModuleConfiguration.txt").start();
         }
         catch (Exception e)
         {
            throw new RuntimeException(e);
         }
      }
   }

   private void setupBipedalSupportPlanarRegionPublisherModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      if (params.isBipedalSupportPlanarRegionPublisherEnabled())
      {
         BipedalSupportPlanarRegionPublisher module = new BipedalSupportPlanarRegionPublisher(robotModel, pubSubImplementation);
         module.start();
         modules.add(module);
      }
   }

   private void setupWalkingPreviewModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isWalkingPreviewToolboxEnabled())
      {
         modules.add(new WalkingControllerPreviewToolboxModule(robotModel, false, pubSubImplementation));
      }
   }

   private void setupHumanoidAvatarREAStateUpdater(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      if (params.isAutoREAStateUpdaterEnabled())
         modules.add(new HumanoidAvatarREAStateUpdater(robotModel, pubSubImplementation));
   }

   protected void connect(PacketCommunicator communicator)
   {
      try
      {
         communicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void printModuleConnectedDebugStatement(PacketDestination destination, String methodName)
   {
      if (DEBUG)
      {
         LogTools.debug(methodName + ": " + destination);
      }
   }

   private void tryToStartModule(ModuleStarter runnable)
   {
      try
      {
         runnable.startModule();
      }
      catch (RuntimeException | IOException e)
      {
         LogTools.error("Failed to start a module in the network processor, stack trace:");
         e.printStackTrace();
      }
   }

   private interface ModuleStarter
   {
      void startModule() throws IOException;
   }

   @Override
   public void closeAndDispose()
   {
      for (CloseableAndDisposable module : modules)
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

      modules.clear();
   }
}
