package us.ihmc.communication.ros2;

import java.io.IOException;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.ros2.RealtimeROS2Publisher;
import us.ihmc.ros2.RealtimeROS2Subscription;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

public class Frame_RealtimeROS2PublisherSubscriberTest
{
   private RealtimeROS2Node realtimeROS2Node;
   private RealtimeROS2Publisher<RobotConfigurationData> publisher;

   public Frame_RealtimeROS2PublisherSubscriberTest()
   {
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "frameTest");
      try
      {
         String topic = "/ihmc/frameTest";
         LogTools.info("Publishing to {}", topic);
         RobotConfigurationDataPubSubType topicDataType = RobotConfigurationData.getPubSubType().get();
         publisher = realtimeROS2Node.createPublisher(topicDataType, topic, ROS2QosProfile.BEST_EFFORT());

         ROS2Subscription<RobotConfigurationData> subscriber = realtimeROS2Node.createSubscription(topicDataType, subscriber2 ->
         {
            LogTools.info("Got from callback");
         }, topic, ROS2QosProfile.BEST_EFFORT());

         int queueSize = 1;
         RealtimeROS2Subscription<RobotConfigurationData> queuedSubscription = realtimeROS2Node.createQueuedSubscription(topicDataType,
                                                                                                                 topic,
                                                                                                                 ROS2QosProfile.BEST_EFFORT(),
                                                                                                                 queueSize);

         ThreadTools.startAThread(() ->
         {
            RobotConfigurationData robotFrameData = new RobotConfigurationData();
            while (true)
            {
               boolean got = queuedSubscription.flushAndGetLatest(robotFrameData);
               if (got)
               {
                  LogTools.info("Got from queued");
               }
               ThreadTools.sleep(1000);
            }
         }, "Subscriber");
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      realtimeROS2Node.spin();

      ThreadTools.startAThread(() ->
      {
         while (true)
         {
            LogTools.info("Publishing...");
            publisher.publish(new RobotConfigurationData());
            ThreadTools.sleep(1000);
         }
      }, "Publisher");

      ROS2Topic<?> typedTopic = new ROS2Topic<>().withPrefix("/ihmc/frameTest").withType(RobotConfigurationData.class);
      LogTools.info("Subscribing to {}", typedTopic.toString());
      FrequencyStatisticPrinter hz = new FrequencyStatisticPrinter();
      ROS2Node node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "hz");
      new IHMCROS2Callback<>(node, typedTopic, ROS2QosProfile.BEST_EFFORT(), message -> hz.ping());

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new Frame_RealtimeROS2PublisherSubscriberTest();
   }
}
