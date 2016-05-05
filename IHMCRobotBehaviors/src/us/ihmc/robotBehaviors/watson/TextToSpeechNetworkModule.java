package us.ihmc.robotBehaviors.watson;

import java.io.IOException;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;

public class TextToSpeechNetworkModule implements PacketConsumer<TextToSpeechPacket>
{
   private final PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.TEXT_TO_SPEECH, 2048, 16384, new IHMCCommunicationKryoNetClassList());
   private final TextToSpeechClient ttsClient = new TextToSpeechClient();
   
   public TextToSpeechNetworkModule()
   {
      try
      {
         packetCommunicator.connect();
         System.out.println("The TextToSpeech Network Module is up and running");
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      packetCommunicator.attachListener(TextToSpeechPacket.class, this);
   }

   @Override
   public void receivedPacket(TextToSpeechPacket packet)
   {
      System.out.println(packet.getTextToSpeak());
      ttsClient.playText(packet.getTextToSpeak());
   }
   
   public static void main(String[] args)
   {
      System.out.println("Starting...");
      new TextToSpeechNetworkModule();
   }
}
