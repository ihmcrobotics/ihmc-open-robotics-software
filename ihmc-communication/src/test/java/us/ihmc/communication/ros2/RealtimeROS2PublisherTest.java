package us.ihmc.communication.ros2;

import controller_msgs.msg.dds.BigVideoPacket;
import controller_msgs.msg.dds.BigVideoPacketPubSubType;
import controller_msgs.msg.dds.VideoPacket;
import controller_msgs.msg.dds.VideoPacketPubSubType;
import std_msgs.msg.dds.Int64;
import std_msgs.msg.dds.Int64PubSubType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.*;
import us.ihmc.tools.time.FrequencyStatisticPrinter;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

import java.io.IOException;

public class RealtimeROS2PublisherTest
{
   private RealtimeROS2Node realtimeROS2Node;
   private RealtimeROS2Publisher<BigVideoPacket> publisher;

   public RealtimeROS2PublisherTest()
   {
      PeriodicThreadSchedulerFactory threadFactory = new PeriodicNonRealtimeThreadSchedulerFactory();
//      try
//      {
//         realtimeROS2Node = new RealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, threadFactory, "NonRealtimeROS2PublishSubscribeExample", "/us/ihmc");
//      }
//      catch (IOException e)
//      {
//         throw new RuntimeException(e);
//      }
         realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "videotest");
      try
      {
         int queueSize = 1;
         String topic = "/ihmc/blackfly/test";
         LogTools.info("Publishing to {}", topic);
         BigVideoPacketPubSubType topicDataType = BigVideoPacket.getPubSubType().get();
//         Int64PubSubType topicDataType = new Int64PubSubType();
         publisher = realtimeROS2Node.createPublisher(topicDataType, topic,
                                                      ROS2QosProfile.BEST_EFFORT());

         ROS2Subscription<BigVideoPacket> subscriber = realtimeROS2Node.createSubscription(topicDataType, subscriber2 ->
         {
            LogTools.info("Got from callback");
         }, topic, ROS2QosProfile.BEST_EFFORT());

         RealtimeROS2Subscription<BigVideoPacket> queuedSubscription = realtimeROS2Node.createQueuedSubscription(topicDataType,
                                                                                                                 topic,
                                                                                                        ROS2QosProfile.BEST_EFFORT(),1);

         ThreadTools.startAThread(() ->
         {
            BigVideoPacket bigVideoPacket = new BigVideoPacket();
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
            publisher.publish(new BigVideoPacket());
            ThreadTools.sleep(1000);
         }
      }, "Publisher");

      ROS2Topic<?> typedTopic = new ROS2Topic<>().withPrefix("/ihmc/blackfly/test").withType(BigVideoPacket.class);
      LogTools.info("Subscribing to {}", typedTopic.toString());
      FrequencyStatisticPrinter hz = new FrequencyStatisticPrinter();
      ROS2Node node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "hz");
      new IHMCROS2Callback<>(node, typedTopic, ROS2QosProfile.BEST_EFFORT(), message -> hz.ping());

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new RealtimeROS2PublisherTest();
   }
}
