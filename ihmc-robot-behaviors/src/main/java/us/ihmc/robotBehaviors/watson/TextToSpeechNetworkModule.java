package us.ihmc.robotBehaviors.watson;

import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.CloseableAndDisposable;

public class TextToSpeechNetworkModule implements CloseableAndDisposable
{
   private final TextToSpeechClient ttsClient = new TextToSpeechClient();
   private final Ros2Node ros2Node;

   public TextToSpeechNetworkModule(PubSubImplementation pubSubImplementation)
   {
      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "ihmc_text_to_speech_node");
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, TextToSpeechPacket.class, ROS2Tools.IHMC_ROOT, s -> receivedPacket(s.takeNextData()));
   }

   public void receivedPacket(TextToSpeechPacket packet)
   {
      LogTools.debug("Received TextToSpeechPacket: " + packet.getTextToSpeakAsString());
      String textToSpeak = packet.getTextToSpeakAsString();
      textToSpeak = "<prosody pitch=\"60Hz\" rate=\"-10%\" volume=\"x-loud\">" + textToSpeak + "</prosody>";
      ttsClient.playText(textToSpeak);
   }

   @Override
   public void closeAndDispose()
   {
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      System.out.println("Starting...");
      new TextToSpeechNetworkModule(PubSubImplementation.FAST_RTPS);
   }
}
