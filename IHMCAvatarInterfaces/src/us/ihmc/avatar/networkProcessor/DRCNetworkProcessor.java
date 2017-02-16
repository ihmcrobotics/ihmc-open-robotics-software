package us.ihmc.avatar.networkProcessor;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map.Entry;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule.FootstepPlanningToolboxModule;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.modules.MultisenseMocapManualCalibrationTestModule;
import us.ihmc.avatar.networkProcessor.modules.RosModule;
import us.ihmc.avatar.networkProcessor.modules.ZeroPoseMockRobotConfigurationDataPublisherModule;
import us.ihmc.avatar.networkProcessor.modules.mocap.IHMCMOCAPLocalizationModule;
import us.ihmc.avatar.networkProcessor.modules.uiConnector.UiConnectionModule;
import us.ihmc.avatar.networkProcessor.quadTreeHeightMap.HeightQuadTreeToolboxModule;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.KryoObjectServer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotBehaviors.watson.TextToSpeechNetworkModule;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationKryoNetClassLists;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.tools.io.printing.PrintTools;

public class DRCNetworkProcessor
{
   private static final IHMCCommunicationKryoNetClassList NET_CLASS_LIST = new IHMCCommunicationKryoNetClassList();
   private final PacketRouter<PacketDestination> packetRouter;
   private final boolean DEBUG = false;

   public DRCNetworkProcessor(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      packetRouter = new PacketRouter<>(PacketDestination.class);
      try
      {
         setupControllerCommunicator(params);
         setupUiModule(params);
         setupSensorModule(robotModel, params);
         setupBehaviorModule(robotModel, params);
         setupHandModules(robotModel, params);
         setupRosModule(robotModel, params);
         setupROSAPIModule(params);
         setupMocapModule(robotModel, params);
         setupZeroPoseRobotConfigurationPublisherModule(robotModel, params);
         setupMultisenseManualTestModule(robotModel, params);
         setupDrillDetectionModule(params);
         setupKinematicsToolboxModule(robotModel, params);
         setupFootstepPlanningToolboxModule(robotModel, params);
         addRobotSpecificModuleCommunicators(params.getRobotSpecificModuleCommunicatorPorts());
         addTextToSpeechEngine(params);
         setupRobotEnvironmentAwarenessModule(params);
         setupHeightQuadTreeToolboxModule(robotModel, params);
         setupLidarScanLogger();
         setupRemoteObjectDetectionFeedbackEndpoint(params);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void addTextToSpeechEngine(DRCNetworkModuleParameters params)
   {
      if (!params.isTextToSpeechModuleEnabled())
         return;

      PacketCommunicator ttsModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.TEXT_TO_SPEECH, NET_CLASS_LIST);
      packetRouter.attachPacketCommunicator(PacketDestination.TEXT_TO_SPEECH, ttsModuleCommunicator);
      try
      {
         ttsModuleCommunicator.connect();
         new TextToSpeechNetworkModule();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      String methodName = "addTextToSpeechEngine";
      printModuleConnectedDebugStatement(PacketDestination.TEXT_TO_SPEECH, methodName);

   }

   private void setupZeroPoseRobotConfigurationPublisherModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      if (params.isZeroPoseRobotConfigurationPublisherEnabled())
      {
         PacketCommunicator zeroPosePublisherCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ZERO_POSE_PRODUCER, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.ZERO_POSE_PRODUCER, zeroPosePublisherCommunicator);
         try
         {
            zeroPosePublisherCommunicator.connect();
            new ZeroPoseMockRobotConfigurationDataPublisherModule(robotModel);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   private void addRobotSpecificModuleCommunicators(HashMap<NetworkPorts, PacketDestination> ports)
   {
      for (Entry<NetworkPorts, PacketDestination> entry : ports.entrySet())
      {
         NetworkPorts port = entry.getKey();
         PacketDestination destinationId = entry.getValue();

         PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(port, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(destinationId, packetCommunicator);
         try
         {
		   	packetCommunicator.connect();
		   }
         catch (IOException e)
         {
   			e.printStackTrace();
	   	}
         printModuleConnectedDebugStatement(destinationId, "addRobotSpecificModuleCommunicators");
      }
   }

   private void setupDrillDetectionModule(DRCNetworkModuleParameters params)
   {
      if (params.isDrillDetectionModuleEnabled())
      {
         PacketCommunicator drillDetectorModuleCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.DRILL_DETECTOR, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.DRILL_DETECTOR, drillDetectorModuleCommunicator);
         try
         {
            drillDetectorModuleCommunicator.connect();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }

         String methodName = "setupDrillDetectionModule";
         printModuleConnectedDebugStatement(PacketDestination.DRILL_DETECTOR, methodName);
      }
   }

   private void setupRemoteObjectDetectionFeedbackEndpoint(DRCNetworkModuleParameters params)
   {
      if (params.isRemoteObjectDetectionFeedbackEnabled())
      {
         PacketCommunicator objectDetectionFeedbackCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.VALVE_DETECTOR_FEEDBACK_PORT, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.OBJECT_DETECTOR, objectDetectionFeedbackCommunicator);
         try
         {
            objectDetectionFeedbackCommunicator.connect();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }

         String methodName = "setupRemoteObjectDetectionFeedbackEndpoint";
         printModuleConnectedDebugStatement(PacketDestination.OBJECT_DETECTOR, methodName);
      }
   }

   private void setupKinematicsToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (!params.isKinematicsToolboxEnabled())
         return;

      new KinematicsToolboxModule(robotModel, params.isKinematicsToolboxVisualizerEnabled());

      PacketCommunicator kinematicsToolboxCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT, NET_CLASS_LIST);
      packetRouter.attachPacketCommunicator(PacketDestination.KINEMATICS_TOOLBOX_MODULE, kinematicsToolboxCommunicator);
      kinematicsToolboxCommunicator.connect();

      String methodName = "setupWholeBodyInverseKinematicsModule";
      printModuleConnectedDebugStatement(PacketDestination.KINEMATICS_TOOLBOX_MODULE, methodName);
   }

   private void setupFootstepPlanningToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (!params.isFootstepPlanningToolboxEnabled())
         return;

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      new FootstepPlanningToolboxModule(robotModel, fullRobotModel, null, params.isFootstepPlanningToolboxVisualizerEnabled());

      PacketCommunicator footstepPlanningToolboxCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.FOOTSTEP_PLANNING_TOOLBOX_MODULE_PORT, NET_CLASS_LIST);
      packetRouter.attachPacketCommunicator(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE, footstepPlanningToolboxCommunicator);
      footstepPlanningToolboxCommunicator.connect();

      String methodName = "setupFootstepPlanningModule";
      printModuleConnectedDebugStatement(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE, methodName);
   }

   private void setupMultisenseManualTestModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      if (params.isMultisenseManualTestModuleEnabled())
      {
         new MultisenseMocapManualCalibrationTestModule(robotModel, params.getRosUri());

         PacketCommunicator multisenseModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.MULTISENSE_MOCAP_MANUAL_CALIBRATION_TEST_MODULE, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.MULTISENSE_TEST_MODULE, multisenseModuleCommunicator);
         try
         {
            multisenseModuleCommunicator.connect();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }

         String methodName = "setupMultisenseManualTestModule";
         printModuleConnectedDebugStatement(PacketDestination.MULTISENSE_TEST_MODULE, methodName);
      }
   }
   
   private void setupMocapModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params)  throws IOException
   {
     if (params.isMocapModuleEnabled())
     {
        new IHMCMOCAPLocalizationModule(robotModel);
        PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.MOCAP_MODULE, NET_CLASS_LIST);
        packetRouter.attachPacketCommunicator(PacketDestination.MOCAP_MODULE, packetCommunicator);
        packetCommunicator.connect();
        
        String methodName = "setupMocapModule";
        printModuleConnectedDebugStatement(PacketDestination.MOCAP_MODULE, methodName);
        
        PrintTools.info("\n\n trying to connect mocap viz server \n\n");
        PacketCommunicator mocapVizPacketCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.MOCAP_MODULE_VIZ, NET_CLASS_LIST);
        mocapVizPacketCommunicator.connect();
        PrintTools.info("\n\n mocap vis packet communicator connected \n\n");
        packetRouter.attachPacketCommunicator(PacketDestination.MOCAP_MODULE_VIZ, mocapVizPacketCommunicator);
     }
   }

   private void setupHandModules(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isHandModuleEnabled())
      {
         for(RobotSide robotSide : RobotSide.values)
         {
            NetworkPorts port = robotSide == RobotSide.LEFT ? NetworkPorts.LEFT_HAND_PORT : NetworkPorts.RIGHT_HAND_PORT;
            PacketCommunicator handModuleCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(port, NET_CLASS_LIST);
            PacketDestination destination = robotSide == RobotSide.LEFT ? PacketDestination.LEFT_HAND : PacketDestination.RIGHT_HAND;
            packetRouter.attachPacketCommunicator(destination, handModuleCommunicator);
            handModuleCommunicator.connect();

            String methodName = "setupHandModules ";
            printModuleConnectedDebugStatement(destination, methodName);
         }
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
            IHMCHumanoidBehaviorManager.createBehaviorModuleForAutomaticDiagnostic(robotModel, logModelProvider, params.isBehaviorVisualizerEnabled(), sensorInformation, params.getTimeToWaitBeforeStartingDiagnostics());
         }
         else
         {
            new IHMCHumanoidBehaviorManager(robotModel, logModelProvider, params.isBehaviorVisualizerEnabled(), sensorInformation);
         }

         PacketCommunicator behaviorModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.BEHAVIOUR_MODULE_PORT, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.BEHAVIOR_MODULE, behaviorModuleCommunicator);
         behaviorModuleCommunicator.connect();


         String methodName = "setupBehaviorModule ";
         printModuleConnectedDebugStatement(PacketDestination.BEHAVIOR_MODULE, methodName);
      }
   }

   private void setupRosModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isRosModuleEnabled())
      {
         new RosModule(robotModel, params.getRosUri(), params.getSimulatedSensorCommunicator());

         PacketCommunicator rosModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_MODULE, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.ROS_MODULE, rosModuleCommunicator);
         rosModuleCommunicator.connect();

         String methodName = "setupRosModule ";
         printModuleConnectedDebugStatement(PacketDestination.ROS_MODULE, methodName);
      }
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

         PacketCommunicator sensorSuiteManagerCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.SENSOR_MANAGER, sensorSuiteManagerCommunicator);
         sensorSuiteManagerCommunicator.connect();

         String methodName = "setupSensorModule ";
         printModuleConnectedDebugStatement(PacketDestination.SENSOR_MANAGER, methodName);
      }
   }

   private void setupUiModule(DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isUiModuleEnabled())
      {
         new UiConnectionModule();

         PacketCommunicator uiModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.UI_MODULE, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.UI, uiModuleCommunicator);
         uiModuleCommunicator.connect();
         String methodName = "setupUiModule ";
         printModuleConnectedDebugStatement(PacketDestination.UI, methodName);
      }
   }

   private void setupROSAPIModule(DRCNetworkModuleParameters params) throws IOException
   {
      if(params.isROSAPICommunicatorEnabled())
      {
         PacketCommunicator rosAPICommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_API_COMMUNICATOR, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.ROS_API, rosAPICommunicator);
         rosAPICommunicator.connect();
         String methodName = "setupROSAPIModule ";
         printModuleConnectedDebugStatement(PacketDestination.ROS_API, methodName);
      }
   }

   private void setupControllerCommunicator(DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isControllerCommunicatorEnabled())
      {
         PacketCommunicator controllerPacketCommunicator;
         if(params.isLocalControllerCommunicatorEnabled())
         {
            PrintTools.info(this, "Connecting to controller using intra process communication");
            controllerPacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, NET_CLASS_LIST);
         }
         else
         {
            System.out.println("Connecting to controller using TCP on " + NetworkParameters.getHost(NetworkParameterKeys.robotController));
            controllerPacketCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(NetworkParameters.getHost(NetworkParameterKeys.robotController), NetworkPorts.CONTROLLER_PORT, NET_CLASS_LIST);
         }

         packetRouter.attachPacketCommunicator(PacketDestination.CONTROLLER, controllerPacketCommunicator);
         controllerPacketCommunicator.connect();

         String methodName = "setupControllerCommunicator ";
         printModuleConnectedDebugStatement(PacketDestination.CONTROLLER, methodName);
      }
   }

   private void setupRobotEnvironmentAwarenessModule(DRCNetworkModuleParameters parameters) throws IOException
   {
      if (parameters.isRobotEnvironmentAwerenessModuleEnabled())
      {
         KryoObjectServer server = new KryoObjectServer(NetworkPorts.REA_MODULE_PORT.getPort(), REACommunicationKryoNetClassLists.getPublicNetClassList());
         server.throwExceptionForUnregisteredPackets(false);
         PacketCommunicator reaCommunicator = PacketCommunicator.createCustomPacketCommunicator(server, REACommunicationKryoNetClassLists.getPublicNetClassList());
         packetRouter.attachPacketCommunicator(PacketDestination.REA_MODULE, reaCommunicator);
         reaCommunicator.connect();
      }
   }

   private void setupHeightQuadTreeToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isHeightQuadTreeToolboxEnabled())
      {
         new HeightQuadTreeToolboxModule(robotModel.createFullRobotModel(), robotModel.getLogModelProvider());

         PacketCommunicator heightQuadTreeToolboxCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.HEIGHT_QUADTREE_TOOLBOX_MODULE_PORT, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.HEIGHT_QUADTREE_TOOLBOX_MODULE, heightQuadTreeToolboxCommunicator);
         heightQuadTreeToolboxCommunicator.connect();

         String methodName = "setupHeightQuadTreeToolboxModule";
         printModuleConnectedDebugStatement(PacketDestination.HEIGHT_QUADTREE_TOOLBOX_MODULE, methodName);
      }
   }

   private void setupLidarScanLogger() throws IOException
   {
      PacketCommunicator lidarScanLoggerCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.LIDAR_SCAN_LOGGER_PORT, NET_CLASS_LIST);
      packetRouter.attachPacketCommunicator(PacketDestination.LIDAR_SCAN_LOGGER, lidarScanLoggerCommunicator);
      lidarScanLoggerCommunicator.connect();

      String methodName = "setupLidarScanLogger";
      printModuleConnectedDebugStatement(PacketDestination.LIDAR_SCAN_LOGGER, methodName);
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

   public PacketRouter<PacketDestination> getPacketRouter()
   {
      return packetRouter;
   }

   private void printModuleConnectedDebugStatement(PacketDestination destination, String methodName)
   {
      if (DEBUG)
      {
         PrintTools.debug(this, methodName + ": " + destination);
      }
   }
}
