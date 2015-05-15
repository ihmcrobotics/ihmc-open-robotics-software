package us.ihmc.darpaRoboticsChallenge.sensors.blackfly;

import java.util.Map;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.sensing.BlackFlyParameterPacket;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.utilities.ros.RosDynamicReconfigure;
import us.ihmc.utilities.ros.RosMainNode;

public class BlackFlyParameterSetter implements PacketConsumer<BlackFlyParameterPacket>
{
   private final PacketCommunicator packetCommunicator;

   RosDynamicReconfigure dynamicReconfigureClient;

   public BlackFlyParameterSetter(RosMainNode rosMainNode, DRCRobotCameraParameters cameraParameters, PacketCommunicator packetCommunicator)
   {
      this.packetCommunicator = packetCommunicator;
      this.dynamicReconfigureClient = new RosDynamicReconfigure(cameraParameters.getRosCameraInfoTopicName().replace("/camera_info", ""), rosMainNode);
      packetCommunicator.attachListener(BlackFlyParameterPacket.class, this);
   }



   public void sendDeviceSettingToUI()
   {
      
      Map<String, Object> params = dynamicReconfigureClient.getParameters();
      packetCommunicator.send(new BlackFlyParameterPacket(false, 
            (Double)params.get("gain"), (Double)params.get("brightness"), (Double)params.get("frame_rate"), (Double)params.get("shutter")));
   }

   public void setBlackFlyParameters(BlackFlyParameterPacket packet)
   {

      System.out.println("object received with gain " + packet.getGain() + " brightness " + packet.getBrightness() + " framerate" + packet.getFrameRate()
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
         dynamicReconfigureClient.setBool("auto_gain", false);
         dynamicReconfigureClient.setDouble("gain", packet.getGain());
         dynamicReconfigureClient.setDouble("brightness", packet.getBrightness());
         dynamicReconfigureClient.setDouble("shutter_speed", packet.getShutter());
      }

      dynamicReconfigureClient.setDouble("frame_rate", packet.getFrameRate());
   }


   public void receivedPacket(BlackFlyParameterPacket packet)
   {
      if (packet.isFromUI()) //avoid hearing my own packet
      {
         setBlackFlyParameters(packet);
         sendDeviceSettingToUI();
      }
   }
}
