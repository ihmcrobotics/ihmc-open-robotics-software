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
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosClockSubscriber;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosMainNode;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.utilities.net.KryoObjectClient;
import us.ihmc.utilities.net.KryoObjectServer;
import us.ihmc.utilities.net.LocalObjectCommunicator;

public class DRCNetworkProcessor
{

   private final KryoObjectClient fieldComputerClient;
   private final KryoObjectServer teamComputerServer;
   private final RobotPoseBuffer robotPoseBuffer;

   
   private final SDFFullRobotModel fullRobotModel;
   private final DRCRobotJointMap jointMap;

   /*
    * This will become a stand-alone application in the final competition. Do
    * NOT pass in objects shared with the DRC simulation!
    */
   
   public DRCNetworkProcessor(URI rosCoreURI)
   {
      this();

      System.out.println("Connecting to ROS");
      RosMainNode rosMainNode;
      rosMainNode = new RosMainNode(rosCoreURI, "darpaRoboticsChallange/networkProcessor");

      RosClockSubscriber timeProvider = new RosClockSubscriber();
      rosMainNode.attachSubscriber("/clock", timeProvider);

      new GazeboCameraReceiver(robotPoseBuffer, DRCConfigParameters.VIDEOSETTINGS, rosMainNode, teamComputerServer, DRCSensorParameters.FIELD_OF_VIEW);
      new GazeboLidarDataReceiver(rosMainNode, robotPoseBuffer, teamComputerServer, fullRobotModel, jointMap, rosCoreURI.toString());
      rosMainNode.execute();
      connect();
   }
   
   public DRCNetworkProcessor(LocalObjectCommunicator scsCommunicator)
   {
      this();
      new SCSCameraDataReceiver(robotPoseBuffer, DRCConfigParameters.VIDEOSETTINGS, scsCommunicator, teamComputerServer);
      new SCSLidarDataReceiver(robotPoseBuffer, scsCommunicator, teamComputerServer, fullRobotModel, jointMap);
      connect();
   }

   private DRCNetworkProcessor()
   {
      fieldComputerClient = new KryoObjectClient(DRCConfigParameters.SCS_MACHINE_IP_ADDRESS, DRCConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT,
            DRCConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_UDP_PORT, new DRCNetClassList());
      fieldComputerClient.setReconnectAutomatically(true);

      teamComputerServer = new KryoObjectServer(DRCConfigParameters.NETWORK_PROCESSOR_TO_UI_TCP_PORT, DRCConfigParameters.NETWORK_PROCESSOR_TO_UI_UDP_PORT,
            new DRCNetClassList());

      robotPoseBuffer = new RobotPoseBuffer(fieldComputerClient, 1000);
      
      jointMap = new DRCRobotJointMap(DRCRobotModel.ATLAS_SANDIA_HANDS, false);  
      JaxbSDFLoader loader = DRCRobotSDFLoader.loadDRCRobot(jointMap);
      fullRobotModel = loader.createFullRobotModel(jointMap);
   }

   private void connect()
   {
      System.out.println("Connecting network processor");
      try
      {
         fieldComputerClient.connect();
         teamComputerServer.connect();
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
