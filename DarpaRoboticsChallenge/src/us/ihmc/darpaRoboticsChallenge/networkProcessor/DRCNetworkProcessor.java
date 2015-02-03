package us.ihmc.darpaRoboticsChallenge.networkProcessor;

import java.io.IOException;
import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.AbstractNetworkProcessor;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketClient;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.communication.packets.sensing.TestbedClientPacket;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.ihmcPerception.depthData.DepthDataFilter;
import us.ihmc.ihmcPerception.depthData.RobotBoundingBoxes;

public class DRCNetworkProcessor extends AbstractNetworkProcessor
{
   private final boolean useSimulatedSensors;
   private final SDFFullRobotModel fullRobotModel;
   private final RobotDataReceiver drcRobotDataReceiver;
   private final RobotPoseBuffer robotPoseBuffer;
   private final DepthDataFilter lidarFilter;

   /*
    * This will become a stand-alone application in the final competition. Do
    * NOT pass in objects shared with the DRC simulation!
    */
   public DRCNetworkProcessor(URI rosUri, DRCRobotModel robotModel)
   {
      this(rosUri, null, null, robotModel, false);
   }

   public DRCNetworkProcessor(PacketCommunicator objectCommunicator, DRCRobotModel robotModel)
   {
      this(null, objectCommunicator, null, robotModel, false);
   }

   public DRCNetworkProcessor(PacketCommunicator scsCommunicator, PacketCommunicator drcNetworkObjectCommunicator, DRCRobotModel robotModel)
   {
      this(null, scsCommunicator, drcNetworkObjectCommunicator, robotModel, false);
   }

   public DRCNetworkProcessor(URI rosUri, PacketCommunicator scsCommunicator, PacketCommunicator controllerCommunicator, DRCRobotModel robotModel,
         boolean startTestbedAlignment)
   {
      if (controllerCommunicator == null)
      {
         String kryoIP = robotModel.getNetworkParameters().getRobotControlComputerIP();

         controllerCommunicator = new KryoPacketClient(kryoIP, NetworkConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT,
               new IHMCCommunicationKryoNetClassList(),PacketDestination.CONTROLLER.ordinal(), PacketDestination.NETWORK_PROCESSOR.ordinal(),"DRCNetworkProcessor");

         if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
         {
            KryoLocalPacketCommunicator behaviorModuleToNetworkProcessorLocalObjectCommunicator = new KryoLocalPacketCommunicator(
                  new IHMCCommunicationKryoNetClassList(), PacketDestination.BEHAVIOR_MODULE.ordinal(), "BehaviorCommunicator");
            
            new IHMCHumanoidBehaviorManager(robotModel, robotModel.getLogModelProvider(), robotModel.getSensorInformation(),
                  behaviorModuleToNetworkProcessorLocalObjectCommunicator, controllerCommunicator);

            try
            {
               controllerCommunicator.connect();
            }
            catch (IOException e)
            {
               // TODO Auto-generated catch block
               e.printStackTrace();
            }

            this.fieldComputerClient = behaviorModuleToNetworkProcessorLocalObjectCommunicator;
         }
         else
         {
            this.fieldComputerClient = controllerCommunicator;
         }
      }
      else
      {
         this.fieldComputerClient = controllerCommunicator;
      }

      useSimulatedSensors = scsCommunicator != null;
      robotPoseBuffer = new RobotPoseBuffer(this.fieldComputerClient, 100000, timestampProvider);
      networkingManager = new DRCNetworkProcessorNetworkingManager(this.fieldComputerClient, timestampProvider, robotModel);
      fullRobotModel = robotModel.createFullRobotModel();

      drcRobotDataReceiver = new RobotDataReceiver(fullRobotModel, null, true);
      RobotBoundingBoxes robotBoundingBoxes = new RobotBoundingBoxes(drcRobotDataReceiver, robotModel.getDRCHandType(), fullRobotModel);
      lidarFilter = new DepthDataFilter(robotBoundingBoxes, fullRobotModel);

      this.fieldComputerClient.attachListener(RobotConfigurationData.class, drcRobotDataReceiver);

      this.fieldComputerClient.attachListener(HandJointAnglePacket.class, new PacketConsumer<HandJointAnglePacket>()
      {
         @Override
         public void receivedPacket(HandJointAnglePacket object)
         {
            networkingManager.getControllerStateHandler().sendPacket(object);
         }
      });

      if (startTestbedAlignment)
      {
         NetworkProcessorTestbedAlignment testbed = new NetworkProcessorTestbedAlignment(networkingManager);
         networkingManager.getControllerCommandHandler().attachListener(TestbedClientPacket.class, testbed);
         new Thread(testbed).start();
      }
      setSensorManager(robotModel.getSensorSuiteManager(rosUri), scsCommunicator, rosUri);
      connect();
   }

   private void setSensorManager(DRCSensorSuiteManager sensorSuiteManager, PacketCommunicator localObjectCommunicator, URI sensorURI)
   {
      if (sensorSuiteManager == null)
         return;

      if (useSimulatedSensors)
      {
         sensorSuiteManager.initializeSimulatedSensors(localObjectCommunicator, fieldComputerClient, robotPoseBuffer, networkingManager, fullRobotModel,
               lidarFilter, sensorURI);
      }
      else
      {
         sensorSuiteManager.initializePhysicalSensors(robotPoseBuffer, networkingManager, fullRobotModel, fieldComputerClient, lidarFilter, sensorURI);
      }
   }

   protected void connect()
   {
      try
      {
         this.fieldComputerClient.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      networkingManager.connect();
   }
}
