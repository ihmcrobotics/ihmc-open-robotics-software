package us.ihmc.robotBehaviors.watson;

import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

public class TextToSpeechNetworkModule
{
   private final TextToSpeechClient ttsClient = new TextToSpeechClient();
   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ihmc_text_to_speech_node");

   public TextToSpeechNetworkModule()
   {
      ROS2Tools.createCallbackSubscription(ros2Node, TextToSpeechPacket.class, ROS2Tools.getDefaultTopicNameGenerator(), s -> receivedPacket(s.takeNextData()));
   }

   public void receivedPacket(TextToSpeechPacket packet)
   {
      PrintTools.debug(this, "Received TextToSpeechPacket: " + packet.getTextToSpeakAsString());
      String textToSpeak = packet.getTextToSpeakAsString();
      textToSpeak = "<prosody pitch=\"60Hz\" rate=\"-10%\" volume=\"x-loud\">" + textToSpeak + "</prosody>";
      ttsClient.playText(textToSpeak);
   }

   public static void main(String[] args)
   {
      System.out.println("Starting...");
      new TextToSpeechNetworkModule();
   }
}
