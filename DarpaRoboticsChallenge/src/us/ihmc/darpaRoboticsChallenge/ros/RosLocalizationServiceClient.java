package us.ihmc.darpaRoboticsChallenge.ros;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import std_srvs.Empty;
import std_srvs.EmptyRequest;
import std_srvs.EmptyResponse;
import us.ihmc.communication.packets.sensing.LocalizationPacket;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosServiceClient;
import ethz_asl_icp_mapper.SetMode;
import ethz_asl_icp_mapper.SetModeRequest;
import ethz_asl_icp_mapper.SetModeResponse;


public class RosLocalizationServiceClient implements ObjectConsumer<LocalizationPacket>
{
   private final RosServiceClient<SetModeRequest, SetModeResponse> localizationClient = new RosServiceClient<SetModeRequest,SetModeResponse>(SetMode._TYPE);
   private final RosServiceClient<EmptyRequest,EmptyResponse> resetClient = new RosServiceClient<EmptyRequest, EmptyResponse>(Empty._TYPE);
   private ObjectCommunicator objectCommunicator;
   private RosMainNode rosMainNode;
   
   public RosLocalizationServiceClient(ObjectCommunicator objectCommunicator, RosMainNode rosMainNode)
   {
      this.rosMainNode = rosMainNode;
      this.objectCommunicator = objectCommunicator;
      
      objectCommunicator.attachListener(LocalizationPacket.class, this);
      
      rosMainNode.attachServiceClient("", localizationClient);
      rosMainNode.attachServiceClient("", resetClient);
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
                     System.out.println("Reference has been reset");
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

   @Override
   public void consumeObject(LocalizationPacket object)
   {
      sendLocalizationMessage(object);
      if (object.getReset())
         sendResetReference(object);
   }
   
}
