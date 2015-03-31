package us.ihmc.darpaRoboticsChallenge.networkProcessor;

import java.io.IOException;

import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicatorMock;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.RosModule;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector.UiConnectionModule;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class DRCNetworkProcessor
{
   private final PacketRouter<PacketDestination> packetRouter;
   private final boolean DEBUG = false;

   public DRCNetworkProcessor(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      packetRouter = new PacketRouter<>(PacketDestination.class);
      
      try
      {
         setupControllerCommunicator(params);
         setupUiModule(robotModel, params);
         setupSensorModule(robotModel, params);
         setupBehaviorModule(robotModel, params);
         setupHandModules(robotModel, params);
         setupRosModule(robotModel, params);
         setupGFEModule(params);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      
   }

   private void setupHandModules(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.useHandModule())
      {
         SideDependentList<? extends HandCommandManager> handCommandModule = robotModel.createHandCommandManager();
         if (handCommandModule == null)
         {
            return;
         }
         
         for(RobotSide robotSide : RobotSide.values)
         {
            NetworkPorts port = robotSide == RobotSide.LEFT ? NetworkPorts.LEFT_HAND_MANAGER_PORT : NetworkPorts.RIGHT_HAND_MANAGER_PORT;
            PacketCommunicatorMock handModuleCommunicator = PacketCommunicatorMock.createIntraprocessPacketCommunicator(port, new IHMCCommunicationKryoNetClassList());
            PacketDestination destination = robotSide == RobotSide.LEFT ? PacketDestination.LEFT_HAND_MANAGER : PacketDestination.RIGHT_HAND_MANAGER;
            packetRouter.attachPacketCommunicator(destination, handModuleCommunicator);
            handModuleCommunicator.connect();
            
            String methodName = "setupHandModules ";
            printModuleConnectedDebugStatement(destination, methodName);
         }
         
            
      }
   }

   private void setupBehaviorModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.useBehaviorModule())
      {
         DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
         LogModelProvider logModelProvider = robotModel.getLogModelProvider();

         if (params.isRunAutomaticDiagnostic())
         {
            IHMCHumanoidBehaviorManager.createBehaviorModuleForAutomaticDiagnostic(robotModel, logModelProvider, params.useBehaviorVisualizer(), sensorInformation, params.getTimeToWaitBeforeStartingDiagnostics());
         }
         else
         {
            new IHMCHumanoidBehaviorManager(robotModel, logModelProvider, params.useBehaviorVisualizer(), sensorInformation);
         }
         
         PacketCommunicatorMock behaviorModuleCommunicator = PacketCommunicatorMock.createIntraprocessPacketCommunicator(NetworkPorts.BEHAVIOUR_MODULE_PORT, new IHMCCommunicationKryoNetClassList());
         packetRouter.attachPacketCommunicator(PacketDestination.BEHAVIOR_MODULE, behaviorModuleCommunicator);
         behaviorModuleCommunicator.connect();


         String methodName = "setupBehaviorModule ";
         printModuleConnectedDebugStatement(PacketDestination.BEHAVIOR_MODULE, methodName);
      }
   }

   private void setupRosModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.useRosModule())
      {
         new RosModule(robotModel, params.getRosUri(), params.getSimulatedSensorCommunicator());

         PacketCommunicatorMock rosModuleCommunicator = PacketCommunicatorMock.createIntraprocessPacketCommunicator(NetworkPorts.ROS_MODULE, new IHMCCommunicationKryoNetClassList());
         packetRouter.attachPacketCommunicator(PacketDestination.ROS_MODULE, rosModuleCommunicator);
         rosModuleCommunicator.connect();
         
         String methodName = "setupRosModule ";
         printModuleConnectedDebugStatement(PacketDestination.ROS_MODULE, methodName);
      }
   }
   
   private void setupSensorModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.useSensorModule())
      {
         DRCSensorSuiteManager sensorSuiteManager = robotModel.getSensorSuiteManager();
         if (params.useSimulatedSensors())
         {
            sensorSuiteManager.initializeSimulatedSensors(params.getSimulatedSensorCommunicator());
         }
         else
         {
            sensorSuiteManager.initializePhysicalSensors(params.getRosUri());
         }
         sensorSuiteManager.connect();
         
         PacketCommunicatorMock sensorSuiteManagerCommunicator = PacketCommunicatorMock.createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER, new IHMCCommunicationKryoNetClassList());
         packetRouter.attachPacketCommunicator(PacketDestination.SENSOR_MANAGER, sensorSuiteManagerCommunicator);
         sensorSuiteManagerCommunicator.connect();
         
         
         String methodName = "setupSensorModule ";
         printModuleConnectedDebugStatement(PacketDestination.SENSOR_MANAGER, methodName);
      }
   }

   private void setupUiModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.useUiModule())
      {
         new UiConnectionModule();
         
         PacketCommunicatorMock uiModuleCommunicator = PacketCommunicatorMock.createIntraprocessPacketCommunicator(NetworkPorts.UI_MODULE, new IHMCCommunicationKryoNetClassList());
         packetRouter.attachPacketCommunicator(PacketDestination.UI, uiModuleCommunicator);
         uiModuleCommunicator.connect();
         String methodName = "setupUiModule ";
         printModuleConnectedDebugStatement(PacketDestination.UI, methodName);
      }
   }

   private void setupGFEModule(DRCNetworkModuleParameters params) throws IOException
   {
      if(params.useGFECommunicator())
      {
         PacketCommunicatorMock gfeCommunicator = PacketCommunicatorMock.createIntraprocessPacketCommunicator(NetworkPorts.GFE_COMMUNICATOR, new IHMCCommunicationKryoNetClassList());
         packetRouter.attachPacketCommunicator(PacketDestination.GFE, gfeCommunicator);
         gfeCommunicator.connect();
         String methodName = "setupGFEModule ";
         printModuleConnectedDebugStatement(PacketDestination.GFE, methodName);
      }
   }

   private void setupControllerCommunicator(DRCNetworkModuleParameters params) throws IOException
   {
      if (params.useController())
      {
         PacketCommunicatorMock controllerPacketCommunicator;
         if(params.useLocalControllerCommunicator())
         {
            System.out.println("Connecting to controller using intra process communication");
            controllerPacketCommunicator = PacketCommunicatorMock.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
         }
         else 
         {
            System.out.println("Connecting to controller using TCP on " + NetworkParameters.getHost(NetworkParameterKeys.robotController));
            controllerPacketCommunicator = PacketCommunicatorMock.createTCPPacketCommunicatorClient(NetworkParameters.getHost(NetworkParameterKeys.robotController), NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
         }
         
         packetRouter.attachPacketCommunicator(PacketDestination.CONTROLLER, controllerPacketCommunicator);
         controllerPacketCommunicator.connect();

         String methodName = "setupControllerCommunicator ";
         printModuleConnectedDebugStatement(PacketDestination.CONTROLLER, methodName);
      }
   }

   protected void connect(PacketCommunicatorMock communicator)
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
//
//   public void addPacketCommunicatorToRouter(PacketDestination destination, PacketCommunicatorMock packetCommunicator)
//   {
//      packetRouter.attachPacketCommunicator(destination, packetCommunicator);
//      connect(packetCommunicator);
//      
//      String methodName = "addPacketCommunicatorToRouter ";
//      printModuleConnectedDebugStatement(destination, methodName);
//   }

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
