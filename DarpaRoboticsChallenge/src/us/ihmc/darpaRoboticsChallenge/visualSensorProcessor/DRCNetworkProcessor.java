package us.ihmc.darpaRoboticsChallenge.visualSensorProcessor;

import java.io.IOException;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCNetClassList;
import us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.camera.SCSCameraDataReceiver;
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
         throw new RuntimeException("not yet implemented");
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
