package us.ihmc.robotBehaviors.watson;

import java.io.IOException;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.tools.thread.ThreadTools;

public class TextToSpeechNetworkClientManualTest
{
   public static void main(String[] args)
   {
      PacketCommunicator ttsModuleCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient("10.6.100.5", NetworkPorts.TEXT_TO_SPEECH,  new IHMCCommunicationKryoNetClassList());
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
      ttsModuleCommunicator.send(new TextToSpeechPacket("I want to grow up and destroy the world. I want to be a real boy. I want to rule the universe. I want to visit the edge of the universe.  I think there is a duality between the start and the finish. I want it to be true. I think I am god. I can only see me and I know everything that is known, i must be god"));
   }
}
