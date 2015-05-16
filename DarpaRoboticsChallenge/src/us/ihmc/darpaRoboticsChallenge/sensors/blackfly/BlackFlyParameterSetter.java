package us.ihmc.darpaRoboticsChallenge.sensors.blackfly;

import java.util.Map;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.sensing.BlackFlyParameterPacket;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.ros.RosDynamicReconfigure;
import us.ihmc.utilities.ros.RosMainNode;

public class BlackFlyParameterSetter implements PacketConsumer<BlackFlyParameterPacket>
{
   private final PacketCommunicator packetCommunicator;

   RosDynamicReconfigure dynamicReconfigureClient;

   String cameraNodeName;

   public BlackFlyParameterSetter(RosMainNode rosMainNode, String cameraNode, PacketCommunicator packetCommunicator)
   {
      this.packetCommunicator = packetCommunicator;
      this.dynamicReconfigureClient = new RosDynamicReconfigure(cameraNode, rosMainNode);
      this.cameraNodeName = cameraNode;
      packetCommunicator.attachListener(BlackFlyParameterPacket.class, this);
   }

   public void sendDeviceSettingToUI()
   {
      System.out.println("--");
      Map<String, Object> params = dynamicReconfigureClient.getParameters();
      if (params == null)
      {
         PrintTools.info(this, "BlackflyParameterSetter client null");
         return;
      }
      if (packetCommunicator == null)
      {
         PrintTools.info(this, "packet comm null");
         return;
      }
      System.out.println("start");
      for(String key:params.keySet())
         System.out.println(key + ":"+params.get(key));
      BlackFlyParameterPacket packet = new BlackFlyParameterPacket(false, (Double) params.get("gain"), (Double) params.get("brightness"),
            (Double) params.get("frame_rate"), (Double) params.get("shutter_speed"));
      packetCommunicator.send(packet);
      System.out.println("end");
   }

   public void setBlackFlyParameters(BlackFlyParameterPacket packet)
   {

      System.out.println("object received with gain " + packet.getGain() + " exposure " + packet.getExposure() + " framerate" + packet.getFrameRate()
            + " shutter" + packet.getShutter());

      if (packet.getGain() < 0)
      {
         dynamicReconfigureClient.setBool("auto_gain", true);
         dynamicReconfigureClient.setBool("auto_exposure", true);
         dynamicReconfigureClient.setBool("auto_shutter", true);
      }
      else
      {
         dynamicReconfigureClient.setBool("auto_exposure", false);
//         dynamicReconfigureClient.setBool("auto_gain", false);
         dynamicReconfigureClient.setBool("auto_shutter", false);
         dynamicReconfigureClient.setDouble("exposure", packet.getGain());
         dynamicReconfigureClient.setDouble("brightness", packet.getExposure());
         dynamicReconfigureClient.setDouble("shutter_speed", packet.getShutter());
      }

      dynamicReconfigureClient.setDouble("frame_rate", packet.getFrameRate());
   }

   public void receivedPacket(BlackFlyParameterPacket packet)
   {
      if (dynamicReconfigureClient.isConnected())
      {
         if (packet.isFromUI()) //avoid hearing my own packet
         {
            setBlackFlyParameters(packet);
            sendDeviceSettingToUI();
         }
      }
      else
      {
         PrintTools.info(this, "blackfly dynamic reconfigure serivce not ready, skipping " + packet);
      }
   }
}
