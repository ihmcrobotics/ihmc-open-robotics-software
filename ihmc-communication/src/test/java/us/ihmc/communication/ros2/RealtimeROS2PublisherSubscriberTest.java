package us.ihmc.communication.ros2;

import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.ImageMessagePubSubType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.QueuedROS2Subscription;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

public class RealtimeROS2PublisherSubscriberTest
{
   private RealtimeROS2Node realtimeROS2Node;
   private ROS2Publisher<ImageMessage> publisher;

   public RealtimeROS2PublisherSubscriberTest()
   {
      realtimeROS2Node = new ROS2NodeBuilder().buildRealtime("videotest");
      String topic = "/ihmc/image/test";
      LogTools.info("Publishing to {}", topic);
      ImageMessagePubSubType topicDataType = ImageMessage.getPubSubType().get();
      publisher = realtimeROS2Node.createPublisher(topicDataType, topic, ROS2QosProfile.BEST_EFFORT());

      ROS2Subscription<ImageMessage> subscriber = realtimeROS2Node.createSubscription(topicDataType, subscriber2 ->
      {
         LogTools.info("Got from callback");
      }, topic, ROS2QosProfile.BEST_EFFORT());

      int queueSize = 1;
      QueuedROS2Subscription<ImageMessage> queuedSubscription = realtimeROS2Node.createQueuedSubscription(topicDataType,
                                                                                                          topic,
                                                                                                          ROS2QosProfile.BEST_EFFORT(),
                                                                                                          queueSize);

      ThreadTools.startAThread(() ->
      {
         ImageMessage bigVideoPacket = new ImageMessage();
         while (true)
         {
            boolean got = queuedSubscription.flushAndGetLatest(bigVideoPacket);
            if (got)
            {
               LogTools.info("Got from queued");
            }
            ThreadTools.sleep(1000);
         }
      }, "Subscriber");

      realtimeROS2Node.spin();

      ThreadTools.startAThread(() ->
      {
         while (true)
         {
            LogTools.info("Publishing...");
            publisher.publish(new ImageMessage());
            ThreadTools.sleep(1000);
         }
      }, "Publisher");

      ROS2Topic<?> typedTopic = new ROS2Topic<>().withPrefix("/ihmc/image/test").withType(ImageMessage.class).withQoS(ROS2QosProfile.BEST_EFFORT());
      LogTools.info("Subscribing to {}", typedTopic.toString());
      FrequencyStatisticPrinter hz = new FrequencyStatisticPrinter();
      ROS2Node node = new ROS2NodeBuilder().build("hz");
      node.createSubscription2(typedTopic, message -> hz.ping());

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new RealtimeROS2PublisherSubscriberTest();
   }
}
