package us.ihmc.robotBehaviors.watson;

import java.io.IOException;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;

public class TextToSpeechNetworkModule implements PacketConsumer<TextToSpeechPacket>
{
   private final PacketCommunicator packetCommunicator;
   private final TextToSpeechClient ttsClient = new TextToSpeechClient();
   
   public TextToSpeechNetworkModule()
   {
//      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.TEXT_TO_SPEECH, 2048, 16384, new IHMCCommunicationKryoNetClassList());
      packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.TEXT_TO_SPEECH, new IHMCCommunicationKryoNetClassList());
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
      PrintTools.debug(this, "Received TextToSpeechPacket: " + packet.getTextToSpeak());
      String textToSpeak = packet.getTextToSpeak();
      textToSpeak = "<prosody pitch=\"60Hz\" rate=\"-10%\" volume=\"x-loud\">" + textToSpeak + "</prosody>";
      ttsClient.playText(textToSpeak);
   }
   
   public static void main(String[] args)
   {
      System.out.println("Starting...");
      new TextToSpeechNetworkModule();
   }
}
