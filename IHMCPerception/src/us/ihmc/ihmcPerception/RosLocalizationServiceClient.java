package us.ihmc.ihmcPerception;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import ethz_asl_icp_mapper.SetMode;
import ethz_asl_icp_mapper.SetModeRequest;
import ethz_asl_icp_mapper.SetModeResponse;
import std_srvs.Empty;
import std_srvs.EmptyRequest;
import std_srvs.EmptyResponse;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPacket;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosServiceClient;


public class RosLocalizationServiceClient implements PacketConsumer<LocalizationPacket>
{
   private final RosServiceClient<SetModeRequest, SetModeResponse> localizationClient = new RosServiceClient<SetModeRequest,SetModeResponse>(SetMode._TYPE);
   private final RosServiceClient<EmptyRequest,EmptyResponse> resetClient = new RosServiceClient<EmptyRequest, EmptyResponse>(Empty._TYPE);
   private RosMainNode rosMainNode;
   private static final boolean DEBUG = false;
   
   public RosLocalizationServiceClient(RosMainNode rosMainNode)
   {
      this.rosMainNode = rosMainNode;
      
      rosMainNode.attachServiceClient("/mapper_humanoid/set_mode", localizationClient);
      rosMainNode.attachServiceClient("/mapper_humanoid/reset", resetClient);
   }
   
   public void sendLocalizationMessage(final LocalizationPacket localizationPacket)
   {
      try
      {
         Thread setupThread = new Thread()
         {
            public void run()
            {
               localizationClient.waitTillConnected();
               
               SetModeRequest request = localizationClient.getMessage();
               
               request.setLocalize(localizationPacket.getToggle());
               request.setMap(localizationPacket.getToggle());
               
               if (DEBUG)
               {
                  System.out.println("Sending message to ROS. Localization = " + localizationPacket.getToggle());
                  System.out.println("Reset = " + localizationPacket.getReset());
               }
               
               localizationClient.call(request, new ServiceResponseListener<SetModeResponse>()
               {
                  
                  @Override
                  public void onSuccess(SetModeResponse response)
                  {
                     boolean toggle = response.getLocalize();
                     System.out.println("Localization is set to: " + toggle);
                  }
                  
                  @Override
                  public void onFailure(RemoteException e)
                  {
                     e.printStackTrace();
                  }
               });
            }
         };
         
         setupThread.start();
      }
      catch (Exception e)
      {
         System.err.println(e.getMessage());
      }
   }
   
   public void sendResetReference(final LocalizationPacket object)
   {
      try
      {
         Thread setupThread = new Thread()
         {
            public void run()
            {
               resetClient.waitTillConnected();
               
               EmptyRequest request = resetClient.getMessage();
               

               
               resetClient.call(request, new ServiceResponseListener<EmptyResponse>()
               {
                  @Override
                  public void onSuccess(EmptyResponse response)
                  {
                     System.out.println("Map has been reset");
                  }
                  
                  @Override
                  public void onFailure(RemoteException e)
                  {
                     e.printStackTrace();
                  }
               });
            }
         };
         
         setupThread.start();
      }
      catch (Exception e)
      {
         System.err.println(e.getMessage());
      }
   }

   public void processLocalizationPacket(LocalizationPacket object)
   {
      if (object.getReset())
         sendResetReference(object);
      else
         sendLocalizationMessage(object);
   }

   public void receivedPacket(LocalizationPacket object)
   {
      processLocalizationPacket(object);
   }
   
}
