package us.ihmc.darpaRoboticsChallenge.networkProcessor;

import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.RosModule;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector.UiConnectionModule;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.ihmcPerception.IHMCPerceptionManager;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class DRCNetworkProcessor
{
   private final PacketRouter packetRouter;
   private final boolean DEBUG = false;

   public DRCNetworkProcessor(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      packetRouter = new PacketRouter();
      ArrayList<PacketCommunicator> communicators = createRequestedModules(robotModel, params);

      for (int i = 0; i < communicators.size(); i++)
      {
         PacketCommunicator packetCommunicator = communicators.get(i);
         if (packetCommunicator != null)
         {
            packetRouter.attachPacketCommunicator(packetCommunicator);
            connect(packetCommunicator);
         }
         else if (DEBUG)
         {
            PrintTools.debug(this, "Null Communicator!");
         }
      }
   }

   private ArrayList<PacketCommunicator> createRequestedModules(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      ArrayList<PacketCommunicator> communicators = new ArrayList<PacketCommunicator>();

      setupControllerCommunicator(params, communicators);
      setupUiModule(robotModel, params, communicators);
      setupSensorModule(robotModel, params, communicators);
      setupPerceptionModule(robotModel, params, communicators);
      setupBehaviorModule(robotModel, params, communicators);
      setupHandModules(robotModel, params, communicators);
      setupRosModule(robotModel, params, communicators);
      setupGFEModule(params, communicators);

      return communicators;
   }
   
   private void setupHandModules(DRCRobotModel robotModel, DRCNetworkModuleParameters params, ArrayList<PacketCommunicator> communicators)
   {
      if (params.useHandModule())
      {
         SideDependentList<? extends HandCommandManager> handCommandModule = robotModel.createHandCommandManager();
         if (handCommandModule == null)
         {
            return;
         }
         
         SideDependentList<PacketCommunicator> handModuleCommunicator = new SideDependentList<PacketCommunicator>(handCommandModule.get(RobotSide.LEFT).getCommunicator(),
               handCommandModule.get(RobotSide.RIGHT).getCommunicator());
         
         for(RobotSide robotSide : RobotSide.values)
         {
            communicators.add(handModuleCommunicator.get(robotSide));
            
            String methodName = "setupHandModules ";
            printModuleConnectedDebugStatement(handModuleCommunicator.get(robotSide), methodName);
         }
      }
   }

   private void setupBehaviorModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params, ArrayList<PacketCommunicator> communicators)
   {
      if (params.useBehaviorModule())
      {
         DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
         LogModelProvider logModelProvider = robotModel.getLogModelProvider();
         IHMCHumanoidBehaviorManager behaviorManager;

         if (params.isRunAutomaticDiagnostic())
         {
            behaviorManager = IHMCHumanoidBehaviorManager.createBehaviorModuleForAutomaticDiagnostic(robotModel, logModelProvider, params.useBehaviorVisualizer(), sensorInformation, params.getTimeToWaitBeforeStartingDiagnostics());
         }
         else
         {
            behaviorManager = new IHMCHumanoidBehaviorManager(robotModel, logModelProvider, params.useBehaviorVisualizer(), sensorInformation);
         }

         PacketCommunicator behaviorModuleCommunicator = behaviorManager.getCommunicator();
         communicators.add(behaviorModuleCommunicator);

         String methodName = "setupBehaviorModule ";
         printModuleConnectedDebugStatement(behaviorModuleCommunicator, methodName);
      }
   }

   private void setupRosModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params, ArrayList<PacketCommunicator> communicators)
   {
      if (params.useRosModule())
      {
         RosModule rosModule = new RosModule(robotModel, params.getRosUri(), params.getSimulatedSensorCommunicator());

         PacketCommunicator rosModuleCommunicator = rosModule.getCommunicator();
         communicators.add(rosModuleCommunicator);

         String methodName = "setupRosModule ";
         printModuleConnectedDebugStatement(rosModuleCommunicator, methodName);
      }
   }

   private void setupPerceptionModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params, ArrayList<PacketCommunicator> communicators)
   {
      if (params.usePerceptionModule())
      {
         IHMCPerceptionManager perceptionModule = new IHMCPerceptionManager();
         PacketCommunicator perceptionModuleCommunicator = perceptionModule.getPerceptionCommunicator();
         communicators.add(perceptionModuleCommunicator);

         String methodName = "setupPerceptionModule ";
         printModuleConnectedDebugStatement(perceptionModuleCommunicator, methodName);
      }
   }

   private void setupSensorModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params, ArrayList<PacketCommunicator> communicators)
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

         PacketCommunicator sensorModuleCommunicator = sensorSuiteManager.getProcessedSensorsCommunicator();
         communicators.add(sensorModuleCommunicator);

         String methodName = "setupSensorModule ";
         printModuleConnectedDebugStatement(sensorModuleCommunicator, methodName);
      }
   }

   private void setupUiModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params, ArrayList<PacketCommunicator> communicators)
   {
      if (params.useUiModule())
      {
         UiConnectionModule uiConnectionModule = new UiConnectionModule();

         KryoPacketCommunicator uiModuleCommunicator = uiConnectionModule.getPacketCommunicator();
         communicators.add(uiModuleCommunicator);

         String methodName = "setupUiModule ";
         printModuleConnectedDebugStatement(uiModuleCommunicator, methodName);
      }
   }

   private void setupGFEModule(DRCNetworkModuleParameters params, ArrayList<PacketCommunicator> communicators)
   {
      if(params.useGFECommunicator())
      {
         PacketCommunicator gfeCommunicator = params.getGFEPacketCommunicator();
         communicators.add(gfeCommunicator);

         String methodName = "setupGFEModule ";
         printModuleConnectedDebugStatement(gfeCommunicator, methodName);
      }
   }

   private void setupControllerCommunicator(DRCNetworkModuleParameters params, ArrayList<PacketCommunicator> communicators)
   {
      if (params.useController())
      {
         PacketCommunicator simulatedControllerCommunicator = params.getControllerCommunicator();
         communicators.add(simulatedControllerCommunicator);

         String methodName = "setupControllerCommunicator ";
         printModuleConnectedDebugStatement(simulatedControllerCommunicator, methodName);
      }
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

   public void addPacketCommunicatorToRouter(PacketCommunicator packetCommunicator)
   {
      packetRouter.attachPacketCommunicator(packetCommunicator);
      connect(packetCommunicator);
      
      String methodName = "addPacketCommunicatorToRouter ";
      printModuleConnectedDebugStatement(packetCommunicator, methodName);
   }

   public PacketRouter getPacketRouter()
   {
      return packetRouter;
   }
   
   private void printModuleConnectedDebugStatement(PacketCommunicator packetCommunicator, String methodName)
   {
      if (DEBUG)
      {
         PrintTools.debug(this, methodName + packetCommunicator.getName() + " " + packetCommunicator.getId());
      }
   }
}
