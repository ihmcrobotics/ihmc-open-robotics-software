package us.ihmc.darpaRoboticsChallenge.networkProcessor;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCNetClassList;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCSensorParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.GazeboCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.GazeboLidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.SCSLidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosMainNode;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.utilities.net.AtomicSettableTimestampProvider;
import us.ihmc.utilities.net.KryoObjectClient;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;

public class DRCNetworkProcessor
{

   private final ObjectCommunicator fieldComputerClient;
   private final AtomicSettableTimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   private final DRCNetworkProcessorNetworkingManager networkingManager;
   private final RobotPoseBuffer robotPoseBuffer;

   
   private final SDFFullRobotModel fullRobotModel;
   private final DRCRobotJointMap jointMap;

   /*
    * This will become a stand-alone application in the final competition. Do
    * NOT pass in objects shared with the DRC simulation!
    */
   public DRCNetworkProcessor(URI rosCoreURI)
   {
      this(rosCoreURI, null);
   }
   
   public DRCNetworkProcessor(URI rosCoreURI, ObjectCommunicator drcNetworkObjectCommunicator)
   {
      this(drcNetworkObjectCommunicator);

      System.out.println("Connecting to ROS");
      RosMainNode rosMainNode;
      rosMainNode = new RosMainNode(rosCoreURI, "darpaRoboticsChallange/networkProcessor");

      new GazeboCameraReceiver(robotPoseBuffer, DRCConfigParameters.VIDEOSETTINGS, rosMainNode, networkingManager, DRCSensorParameters.FIELD_OF_VIEW);
      new GazeboLidarDataReceiver(rosMainNode, robotPoseBuffer, networkingManager, fullRobotModel, jointMap, rosCoreURI.toString());
      rosMainNode.execute();
      connect();
   }
   
   public DRCNetworkProcessor(LocalObjectCommunicator scsCommunicator, ObjectCommunicator drcNetworkObjectCommunicator)
   {
      this(drcNetworkObjectCommunicator);
      new SCSCameraDataReceiver(robotPoseBuffer, DRCConfigParameters.VIDEOSETTINGS, scsCommunicator, networkingManager);
      new SCSLidarDataReceiver(robotPoseBuffer, scsCommunicator, networkingManager, fullRobotModel, jointMap);
      connect();
   }

   private DRCNetworkProcessor(ObjectCommunicator fieldComputerClient)
   {
      if(fieldComputerClient == null)
      {
         this.fieldComputerClient = new KryoObjectClient(DRCConfigParameters.SCS_MACHINE_IP_ADDRESS, DRCConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT,
               new DRCNetClassList());
         ((KryoObjectClient)this.fieldComputerClient).setReconnectAutomatically(true);
      }
      else
      {
         this.fieldComputerClient = fieldComputerClient;
      }
      
      robotPoseBuffer = new RobotPoseBuffer(this.fieldComputerClient, 1000, timestampProvider);
      networkingManager = new DRCNetworkProcessorNetworkingManager(this.fieldComputerClient, timestampProvider);
      
      jointMap = new DRCRobotJointMap(DRCRobotModel.ATLAS_SANDIA_HANDS, false);  
      JaxbSDFLoader loader = DRCRobotSDFLoader.loadDRCRobot(jointMap, true);
      fullRobotModel = loader.createFullRobotModel(jointMap);
   }

   private void connect()
   {
      System.out.println("Connecting network processor");
      try
      {
         fieldComputerClient.connect();
         networkingManager.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   
   public static void main(String[] args) throws URISyntaxException
   {
      if(args.length == 1)
      {
         new DRCNetworkProcessor(new URI(args[0]));
      }
      else
      {
         new DRCNetworkProcessor(new URI(DRCConfigParameters.ROS_MASTER_URI));
      }
   }
}
