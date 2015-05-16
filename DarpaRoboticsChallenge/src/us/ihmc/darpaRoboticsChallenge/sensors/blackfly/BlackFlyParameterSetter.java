package us.ihmc.darpaRoboticsChallenge.sensors.blackfly;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.sensing.BlackFlyParameterPacket;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.ros.RosDynamicReconfigure;
import us.ihmc.utilities.ros.RosMainNode;

public class BlackFlyParameterSetter implements PacketConsumer<BlackFlyParameterPacket>
{
   private final PacketCommunicator packetCommunicator;

   RosDynamicReconfigure dynamicReconfigureClient;

   String cameraNodeName;
   RobotSide side;

   public BlackFlyParameterSetter(RosMainNode rosMainNode, RobotSide side, String cameraNode, PacketCommunicator packetCommunicator)
   {
      this.packetCommunicator = packetCommunicator;
      this.dynamicReconfigureClient = new RosDynamicReconfigure(cameraNode, rosMainNode);
      this.side = side;
      this.cameraNodeName = cameraNode;
      packetCommunicator.attachListener(BlackFlyParameterPacket.class, this);
   }

   public void sendDeviceSettingToUI(Map<String, Object> params)
   {
      for (String key : params.keySet())
         System.out.println(key + ":" + params.get(key));
      BlackFlyParameterPacket packet = new BlackFlyParameterPacket(false, (Double) params.get("gain"), (Double) params.get("brightness"),
            (Double) params.get("frame_rate"), (Double) params.get("shutter_speed"), (Boolean) params.get("auto_exposure"), (Boolean) params.get("auto_gain"),
            (Boolean) params.get("auto_shutter"), side);
      packetCommunicator.send(packet);
   }

   public Map<String, Object> setBlackFlyParameters(BlackFlyParameterPacket packet)
   {

      System.out.println("object received with " + packet);
      Map<String, Object> parameters = new HashMap<>();
      parameters.put("auto_exposure", packet.isAutoExposure());
      parameters.put("auto_gain", packet.isAutoGain());
      parameters.put("auto_shutter", packet.isAutoShutter());
      parameters.put("exposure", packet.getGain());
      parameters.put("brightness", packet.getExposure());
      parameters.put("shutter_speed", packet.getShutter());
      parameters.put("frame_rate", packet.getFrameRate());
      return dynamicReconfigureClient.setParameters(parameters);
   }

   public void onConnect()
   {
      sendDeviceSettingToUI(dynamicReconfigureClient.getParameters());
   }

   public void receivedPacket(BlackFlyParameterPacket packet)
   {
      if (dynamicReconfigureClient.isConnected())
      {
         if (packet.isFromUI() && packet.getSide() == this.side) //avoid hearing my own packet
         {
            Map<String, Object> result = setBlackFlyParameters(packet);
            sendDeviceSettingToUI(result);
         }
      }
      else
      {
         PrintTools.info(this, "blackfly dynamic reconfigure serivce not ready, skipping " + packet);
      }
   }
}
