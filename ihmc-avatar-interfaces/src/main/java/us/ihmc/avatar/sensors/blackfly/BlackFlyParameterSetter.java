package us.ihmc.avatar.sensors.blackfly;

import java.util.HashMap;
import java.util.Map;

import controller_msgs.msg.dds.BlackFlyParameterPacket;
import controller_msgs.msg.dds.UIConnectedPacket;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.ros.RosDynamicReconfigure;
import us.ihmc.utilities.ros.RosMainNode;

public class BlackFlyParameterSetter implements PacketConsumer<BlackFlyParameterPacket>
{
   private final PacketCommunicator packetCommunicator;
   private final boolean DEBUG = false;
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
      setupUIOnConnectResponder();

   }

   public void sendDeviceSettingToUI(Map<String, Object> params)
   {
      BlackFlyParameterPacket packet = HumanoidMessageTools.createBlackFlyParameterPacket(false, (Double) params.get("gain"), (Double) params.get("exposure"),
            (Double) params.get("frame_rate"), (Double) params.get("shutter_speed"), (Boolean) params.get("auto_exposure"), (Boolean) params.get("auto_gain"),
            (Boolean) params.get("auto_shutter"), side);
      packetCommunicator.send(packet);
      PrintTools.debug(DEBUG, this, "packet to UI " + packet);
   }

   public Map<String, Object> setBlackFlyParameters(BlackFlyParameterPacket packet)
   {

      PrintTools.debug(DEBUG, this, "packet fr UI " + packet);
      Map<String, Object> parameters = new HashMap<>();
      parameters.put("auto_exposure", packet.getAutoExposure());
      parameters.put("auto_gain", packet.getAutoGain());
      parameters.put("auto_shutter", packet.getAutoShutter());
      parameters.put("exposure", packet.getExposure());
      parameters.put("gain", packet.getGain());
      parameters.put("shutter_speed", packet.getShutter());
      parameters.put("frame_rate", packet.getFrameRate());
      return dynamicReconfigureClient.setParameters(parameters);
   }

   public void setupUIOnConnectResponder()
   {

      packetCommunicator.attachListener(UIConnectedPacket.class, new PacketConsumer<UIConnectedPacket>()
      {
         @Override
         public void receivedPacket(UIConnectedPacket packet)
         {
            sendDeviceSettingToUI(dynamicReconfigureClient.getParameters());
            PrintTools.info(this, "UIConnected - send parameters");
         }
      });
   }

   public void receivedPacket(final BlackFlyParameterPacket packet)
   {

      if (dynamicReconfigureClient.isConnected())
      {
         if (packet.getFromUi() && packet.getRobotSide() == BlackFlyParameterSetter.this.side.toByte()) //avoid hearing my own packet
         {
            new Thread("BlackflyDynamicReconfigureSetter")
            {
               public void run()
               {
                  Map<String, Object> result = setBlackFlyParameters(packet);
                  sendDeviceSettingToUI(result);
               }
            }.start();
         }
         else
         {
            PrintTools.info(this, "blackfly dynamic reconfigure serivce not ready, skipping " + packet);
         }

      }

   }
}
