package us.ihmc.communication.ros2;

import org.junit.jupiter.api.Disabled;
import perception_msgs.ImageMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

@Disabled("Manual interactive test — blocks with sleepForever()")
public class RealtimeROS2PublisherSubscriberTest
{
   private AsyncROS2Node realtimeROS2Node;
   private ROS2Publisher<ImageMessage> publisher;

   public RealtimeROS2PublisherSubscriberTest()
   {
      realtimeROS2Node = new AsyncROS2Node("videotest");
      ROS2Topic<ImageMessage> topic = new ROS2Topic<ImageMessage>().prependedWith("/ihmc/image/test")
                                                                     .withType(ImageMessage.class)
                                                                     .withQoS(ROS2QoSProfile.BEST_EFFORT);
      LogTools.info("Publishing to {}", topic);
      publisher = realtimeROS2Node.createPublisher(topic);

      ROS2Subscription<ImageMessage> subscriber = realtimeROS2Node.createSubscription(topic, reader ->
      {
         ImageMessage message = reader.read();
         LogTools.info("Got from callback");
      });
      ThreadTools.startAThread(() ->
      {
         while (true)
         {
            LogTools.info("Publishing...");
            publisher.publish(new ImageMessage());
            ThreadTools.sleep(1000);
         }
      }, "Publisher");

      LogTools.info("Subscribing to {}", topic.toString());
      FrequencyStatisticPrinter hz = new FrequencyStatisticPrinter();
      ROS2Node node = new ROS2Node("hz");
      node.createSubscription(topic, reader -> hz.ping());

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new RealtimeROS2PublisherSubscriberTest();
   }
}
