package us.ihmc.darpaRoboticsChallenge.sensors.blackfly;

import org.ros.exception.RemoteException;
import org.ros.node.NodeConfiguration;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceResponseListener;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicatorMock;
import us.ihmc.communication.packets.sensing.BlackFlyParameterPacket;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosServiceClient;
import dynamic_reconfigure.DoubleParameter;
import dynamic_reconfigure.Reconfigure;
import dynamic_reconfigure.ReconfigureRequest;
import dynamic_reconfigure.ReconfigureResponse;

public class BlackFlyParameterSetter implements PacketConsumer<BlackFlyParameterPacket>
{
   private static double gain;
   private static double brightness;
   private static double frameRate;
   private static double Shutter;
   private final RosServiceClient<ReconfigureRequest, ReconfigureResponse> blackFlyClient;
   private final RosMainNode rosMainNode;
   private final PacketCommunicatorMock packetCommunicator;
   private ParameterTree params;

   public BlackFlyParameterSetter(RosMainNode rosMainNode, PacketCommunicatorMock packetCommunicator)
   {
      this.rosMainNode = rosMainNode;
      this.packetCommunicator = packetCommunicator;
      blackFlyClient = new RosServiceClient<ReconfigureRequest, ReconfigureResponse>(Reconfigure._TYPE);
      rosMainNode.attachServiceClient("blackfly/set_parameters", blackFlyClient);
      
      packetCommunicator.attachListener(BlackFlyParameterPacket.class, this);
   }

   public void handleBlackFlyParameters(BlackFlyParameterPacket object)
   {
      if (object.isFromUI())
      {
         if (rosMainNode.isStarted())
         {
            params = rosMainNode.getParameters();
            send();

         }
      }
      else
         setBlackFlyParameters(object);

   }

   private void send()
   {
      if (params == null)
      {
         //System.out.println("params are null");
         return;
      }

      packetCommunicator.send(
            new BlackFlyParameterPacket(false, params.getDouble("/blackfly/prop_gain"), params.getDouble("/blackfly/prop_brightness"), params
                  .getDouble("/blackfly/prop_frame_rate"), params.getDouble("/blackfly/prop_shutter")));
   }

   public void initializeParameterListeners()
   {

    //  System.out.println("-----------initialising blackfly parameter listeners-------------------");

      rosMainNode.attachParameterListener("/blackfly/prop_gain", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
           // System.out.println("new blackfly gain received");
            send();
         }
      });
      rosMainNode.attachParameterListener("/blackfly/prop_brightness", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
           // System.out.println("new blackfly brightness received");
            send();
         }
      });

      rosMainNode.attachParameterListener("/blackfly/prop_frame_rate", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
           // System.out.println("new blackfly fram rate received");
            send();
         }
      });
      rosMainNode.attachParameterListener("/blackfly/prop_shutter", new ParameterListener()
      {

         @Override
         public void onNewValue(Object value)
         {
           // System.out.println("new blackfly shutter received");
            send();
         }
      });

   }
   
   
   public void setBlackFlyParameters(BlackFlyParameterPacket object)
   {
     
      //System.out.println("object received with gain "+ object.getGain()+" brightness "+ object.getBrightness()+" framerate"+object.getFrameRate()+" shutter"+ object.getShutter());
     
      
      
      blackFlyClient.waitTillConnected();
      ReconfigureRequest request = blackFlyClient.getMessage();
      if(object.getGain() != gain){
      gain = object.getGain();
      DoubleParameter gainParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
      gainParam.setName("prop_gain");
      gainParam.setValue(gain);
      request.getConfig().getDoubles().add(gainParam);
      }
      
      if(object.getBrightness() != brightness){
         brightness = object.getBrightness();
      DoubleParameter brightnessParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
      brightnessParam.setName("prop_brightness");
      brightnessParam.setValue(brightness);
      request.getConfig().getDoubles().add(brightnessParam);
      }
      
      if(object.getFrameRate() != frameRate){
      frameRate = object.getFrameRate();
      DoubleParameter frameRateParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
      frameRateParam.setName("prop_frame_rate");
      frameRateParam.setValue(frameRate);
      request.getConfig().getDoubles().add(frameRateParam);
      }
      
      if(object.getShutter() != Shutter){
         Shutter = object.getShutter();
         DoubleParameter shutterParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
         shutterParam.setName("prop_shutter");
         shutterParam.setValue(Shutter);
         request.getConfig().getDoubles().add(shutterParam);
         }
         
          
//      blackFlyClient.call(request, new ServiceResponseListener<ReconfigureResponse>()
//            {
//
//               public void onSuccess(ReconfigureResponse response)
//               {
//                  System.out.println("successful" + response.getConfig().getDoubles().get(0).getValue());
//               }
//
//               public void onFailure(RemoteException e)
//               {
//                  e.printStackTrace();
//               }
//            });
   }

   
   public void setFishEyeFrameRate(double frameRate)
   {
      
      blackFlyClient.waitTillConnected();
      System.out.println("setting the framer rate for fish eye");
      ReconfigureRequest request = blackFlyClient.getMessage();
      DoubleParameter frameRateDoubleParam = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(DoubleParameter._TYPE);
      frameRateDoubleParam.setName("prop_frame_rate");
      frameRateDoubleParam.setValue(frameRate);
      request.getConfig().getDoubles().add(frameRateDoubleParam);
           
      blackFlyClient.call(request, new ServiceResponseListener<ReconfigureResponse>()
      {

         public void onSuccess(ReconfigureResponse response)
         {
            System.out.println("success" + response.getConfig().getDoubles().get(1).getValue());
         }

         public void onFailure(RemoteException e)
         {
            throw new RuntimeException(e);
         }
      });
   }

   public void receivedPacket(BlackFlyParameterPacket object)
   {
      handleBlackFlyParameters(object);
   }
}
