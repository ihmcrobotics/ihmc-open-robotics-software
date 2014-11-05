package us.ihmc.darpaRoboticsChallenge.networkProcessor;

import java.io.IOException;
import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.AbstractNetworkProcessor;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.communication.packets.sensing.TestbedClientPacket;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.DepthDataFilter;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.RobotBoundingBoxes;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.utilities.net.KryoLocalObjectCommunicator;
import us.ihmc.utilities.net.KryoObjectClient;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.ObjectConsumer;

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
   
   public DRCNetworkProcessor(LocalObjectCommunicator objectCommunicator, DRCRobotModel robotModel)
   {
      this(null, objectCommunicator, null, robotModel, false);
   }

   public DRCNetworkProcessor(LocalObjectCommunicator scsCommunicator, ObjectCommunicator drcNetworkObjectCommunicator, DRCRobotModel robotModel)
   {
      this(null, scsCommunicator, drcNetworkObjectCommunicator, robotModel, false);
   }
   
   public DRCNetworkProcessor(URI rosUri, LocalObjectCommunicator scsCommunicator, ObjectCommunicator fieldComputerClient, DRCRobotModel robotModel, boolean startTestbedAlignment)
   {
      if (fieldComputerClient == null)
      {
    	  String kryoIP = robotModel.getNetworkParameters().getRobotControlComputerIP();
       	  fieldComputerClient = new KryoObjectClient(kryoIP, NetworkConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT,
    			  new IHMCCommunicationKryoNetClassList());
    	  ((KryoObjectClient) fieldComputerClient).setReconnectAutomatically(true);    
    	  
          if(NetworkConfigParameters.USE_BEHAVIORS_MODULE)
          {
        	  KryoLocalObjectCommunicator behaviorModuleToNetworkProcessorLocalObjectCommunicator = new KryoLocalObjectCommunicator(new IHMCCommunicationKryoNetClassList());
              new IHMCHumanoidBehaviorManager(robotModel.createFullRobotModel(), behaviorModuleToNetworkProcessorLocalObjectCommunicator, fieldComputerClient);
             
              try {
            	  fieldComputerClient.connect();
              } catch (IOException e) {
            	  // TODO Auto-generated catch block
            	  e.printStackTrace();
              }
              
              this.fieldComputerClient = behaviorModuleToNetworkProcessorLocalObjectCommunicator;
          }
          else
          {
        	  this.fieldComputerClient = fieldComputerClient;
          }
      }
      else
      {
         this.fieldComputerClient = fieldComputerClient;
      }
            
      useSimulatedSensors = scsCommunicator != null;
      robotPoseBuffer = new RobotPoseBuffer(this.fieldComputerClient, 1000, timestampProvider);
      networkingManager = new DRCNetworkProcessorNetworkingManager(this.fieldComputerClient, timestampProvider, robotModel);
      fullRobotModel = robotModel.createFullRobotModel();
      drcRobotDataReceiver = new RobotDataReceiver(fullRobotModel, null, true);
      RobotBoundingBoxes robotBoundingBoxes = new RobotBoundingBoxes(drcRobotDataReceiver, robotModel, fullRobotModel);
      lidarFilter = new DepthDataFilter(robotBoundingBoxes, fullRobotModel);
      
      this.fieldComputerClient.attachListener(RobotConfigurationData.class, drcRobotDataReceiver);
      this.fieldComputerClient.attachListener(HandJointAnglePacket.class, new ObjectConsumer<HandJointAnglePacket>()
      {
         @Override
         public void consumeObject(HandJointAnglePacket object)
         {
            networkingManager.getControllerStateHandler().sendSerializableObject(object);
         }
      });

      if (startTestbedAlignment)
      {
         NetworkProcessorTestbedAlignment testbed = new NetworkProcessorTestbedAlignment(networkingManager);
         networkingManager.getControllerCommandHandler().attachListener(TestbedClientPacket.class, testbed);
         new Thread(testbed).start();
      }
      setSensorManager(robotModel.getSensorSuiteManager(rosUri), scsCommunicator, rosUri, robotModel);
      connect();
   }

   private void setSensorManager(DRCSensorSuiteManager sensorSuiteManager, LocalObjectCommunicator localObjectCommunicator, URI sensorURI, DRCRobotModel robotModel)
   {
      if (useSimulatedSensors)
      {
         sensorSuiteManager.initializeSimulatedSensors(localObjectCommunicator, fieldComputerClient, robotPoseBuffer, networkingManager, fullRobotModel, lidarFilter, sensorURI, robotModel);
      }
      else
      {
         sensorSuiteManager.initializePhysicalSensors(robotPoseBuffer,networkingManager,fullRobotModel,fieldComputerClient, lidarFilter, sensorURI, robotModel);
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
