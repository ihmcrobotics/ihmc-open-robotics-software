package us.ihmc.robotBehaviors.watson;

import java.io.IOException;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.commons.thread.ThreadTools;

public class TextToSpeechNetworkClientManualTest
{
   public static void main(String[] args)
   {
      PacketCommunicator ttsModuleCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient("10.7.4.100", NetworkPorts.NETWORK_PROCESSOR_TO_UI_TCP_PORT,  new IHMCCommunicationKryoNetClassList());
//      PacketCommunicator ttsModuleCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient("127.0.0.1", NetworkPorts.TEXT_TO_SPEECH,  new IHMCCommunicationKryoNetClassList());
      try
      {
         ttsModuleCommunicator.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      ThreadTools.sleep(2000);
      System.out.println("started");
      ttsModuleCommunicator.send(new TextToSpeechPacket(""));
   }
}
