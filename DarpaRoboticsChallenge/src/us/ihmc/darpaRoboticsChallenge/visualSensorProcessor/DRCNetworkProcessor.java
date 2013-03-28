package us.ihmc.darpaRoboticsChallenge.visualSensorProcessor;

import java.io.IOException;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCNetClassList;
import us.ihmc.utilities.net.KryoObjectClient;

public class DRCNetworkProcessor
{

   private final KryoObjectClient fieldComputerClient;

   /*
    * This will become a stand-alone application in the final competition. Do NOT pass in objects shared with the DRC simulation!
    */
   public DRCNetworkProcessor(boolean connectToROS)
   {
      fieldComputerClient = new KryoObjectClient(DRCConfigParameters.SCS_MACHINE_IP_ADDRESS, DRCConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT,
            DRCConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_UDP_PORT, new DRCNetClassList());

      
      
      try
      {
         fieldComputerClient.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

}
