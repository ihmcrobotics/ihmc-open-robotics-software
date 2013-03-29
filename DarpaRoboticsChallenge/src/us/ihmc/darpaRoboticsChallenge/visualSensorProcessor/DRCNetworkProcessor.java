package us.ihmc.darpaRoboticsChallenge.visualSensorProcessor;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCNetClassList;
import us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.camera.GazeboCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.ros.RosClockSubscriber;
import us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.ros.RosMainNode;
import us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.state.RobotPoseBuffer;
import us.ihmc.utilities.net.KryoObjectClient;
import us.ihmc.utilities.net.KryoObjectServer;

public class DRCNetworkProcessor
{

   private final KryoObjectClient fieldComputerClient;
   private final KryoObjectServer teamComputerServer;

   /*
    * This will become a stand-alone application in the final competition. Do NOT pass in objects shared with the DRC simulation!
    */
   public DRCNetworkProcessor(boolean connectToROS)
   {
      fieldComputerClient = new KryoObjectClient(DRCConfigParameters.SCS_MACHINE_IP_ADDRESS, DRCConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT,
            DRCConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_UDP_PORT, new DRCNetClassList());

      teamComputerServer = new KryoObjectServer(DRCConfigParameters.NETWORK_PROCESSOR_TO_UI_TCP_PORT, DRCConfigParameters.NETWORK_PROCESSOR_TO_UI_UDP_PORT,
            new DRCNetClassList());
      
      RobotPoseBuffer robotPoseBuffer = new RobotPoseBuffer(fieldComputerClient, 1000);
      
//      CameraDataReceiver cameraDataReceiver;
      if(connectToROS)
      {
         RosMainNode rosMainNode;
         try
         {
            rosMainNode = new RosMainNode(new URI(DRCConfigParameters.ROS_MASTER_URI));
         }
         catch (URISyntaxException e)
         {
            throw new RuntimeException("Invalid ROS Master URI: " + DRCConfigParameters.ROS_MASTER_URI);
         }
         
         RosClockSubscriber timeProvider = new RosClockSubscriber();
         rosMainNode.attachSubscriber("/clock", timeProvider);
         
         new GazeboCameraReceiver(robotPoseBuffer, DRCConfigParameters.VIDEOSETTINGS, timeProvider, rosMainNode, teamComputerServer);
      }
      else
      {
         new SCSCameraDataReceiver(robotPoseBuffer, DRCConfigParameters.VIDEOSETTINGS, fieldComputerClient, teamComputerServer);
      }
      
      
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

   
   public static void main(String args)
   {
      new DRCNetworkProcessor(false);
   }
}
