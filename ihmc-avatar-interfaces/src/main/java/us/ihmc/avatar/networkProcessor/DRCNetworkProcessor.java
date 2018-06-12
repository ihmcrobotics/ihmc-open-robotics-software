package us.ihmc.avatar.networkProcessor;

import java.io.IOException;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule.FootstepPlanningToolboxModule;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.modules.RosModule;
import us.ihmc.avatar.networkProcessor.modules.ZeroPoseMockRobotConfigurationDataPublisherModule;
import us.ihmc.avatar.networkProcessor.modules.mocap.IHMCMOCAPLocalizationModule;
import us.ihmc.avatar.networkProcessor.modules.mocap.MocapPlanarRegionsListManager;
import us.ihmc.avatar.networkProcessor.quadTreeHeightMap.HeightQuadTreeToolboxModule;
import us.ihmc.avatar.networkProcessor.rrtToolboxModule.WholeBodyTrajectoryToolboxModule;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotBehaviors.watson.TextToSpeechNetworkModule;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;

public class DRCNetworkProcessor
{
   private final boolean DEBUG = false;

   public DRCNetworkProcessor(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      tryToStartModule(() -> setupSensorModule(robotModel, params));
      tryToStartModule(() -> setupBehaviorModule(robotModel, params));
      tryToStartModule(() -> setupRosModule(robotModel, params));
      tryToStartModule(() -> setupMocapModule(robotModel, params));
      tryToStartModule(() -> setupZeroPoseRobotConfigurationPublisherModule(robotModel, params));
      tryToStartModule(() -> setupConstrainedWholebodyPlanningToolboxModule(robotModel, params));
      tryToStartModule(() -> setupKinematicsToolboxModule(robotModel, params));
      tryToStartModule(() -> setupFootstepPlanningToolboxModule(robotModel, params));
      tryToStartModule(() -> addTextToSpeechEngine(params));
      tryToStartModule(() -> setupHeightQuadTreeToolboxModule(robotModel, params));
   }

   private void addTextToSpeechEngine(DRCNetworkModuleParameters params)
   {
      if (params.isTextToSpeechModuleEnabled())
         new TextToSpeechNetworkModule();
   }

   private void setupZeroPoseRobotConfigurationPublisherModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      if (params.isZeroPoseRobotConfigurationPublisherEnabled())
         new ZeroPoseMockRobotConfigurationDataPublisherModule(robotModel);
   }

   private void setupConstrainedWholebodyPlanningToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (!params.isConstrainedWholeBodyPlanningToolboxEnabled())
         return;

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      new WholeBodyTrajectoryToolboxModule(robotModel, fullRobotModel, null, params.isConstrainedWholeBodyToolboxVisualizerEnabled());
      PrintTools.info("setupConstrainedWholebodyPlanningToolboxModule");
   }

   private void setupKinematicsToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (!params.isKinematicsToolboxEnabled())
         return;
      new KinematicsToolboxModule(robotModel, params.isKinematicsToolboxVisualizerEnabled());
   }

   private void setupFootstepPlanningToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (!params.isFootstepPlanningToolboxEnabled())
         return;

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      new FootstepPlanningToolboxModule(robotModel, fullRobotModel, null, params.isFootstepPlanningToolboxVisualizerEnabled());
   }

   private void setupMocapModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isMocapModuleEnabled())
      {
         MocapPlanarRegionsListManager planarRegionsListManager = new MocapPlanarRegionsListManager();

         Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ihmc_mocap_localization_node");
         ROS2Tools.createCallbackSubscription(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator,
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
         DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
         LogModelProvider logModelProvider = robotModel.getLogModelProvider();

         if (params.isAutomaticDiagnosticEnabled())
         {
            IHMCHumanoidBehaviorManager.createBehaviorModuleForAutomaticDiagnostic(robotModel, robotModel, logModelProvider,
                                                                                   params.isBehaviorVisualizerEnabled(), sensorInformation,
                                                                                   params.getTimeToWaitBeforeStartingDiagnostics());
         }
         else
         {
            new IHMCHumanoidBehaviorManager(robotModel, robotModel, logModelProvider, params.isBehaviorVisualizerEnabled(), sensorInformation);
         }

         String methodName = "setupBehaviorModule ";
         printModuleConnectedDebugStatement(PacketDestination.BEHAVIOR_MODULE, methodName);
      }
   }

   private void setupRosModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isRosModuleEnabled())
         new RosModule(robotModel, params.getRosUri(), params.getSimulatedSensorCommunicator());
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
         new HeightQuadTreeToolboxModule(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider());
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
         PrintTools.debug(this, methodName + ": " + destination);
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
         PrintTools.error(this, "Failed to start a module in the network processor, stack trace:");
         e.printStackTrace();
      }
   }

   private interface ModuleStarter
   {
      void startModule() throws IOException;
   }
}
