package us.ihmc.communication.ros2;

import java.io.IOException;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.ros2.RealtimeROS2Subscription;

public class FrameRealtimeROS2PublisherSubscriberTest
{
   private RealtimeROS2Node realtimeROS2Node;
   private ROS2PublisherBasics<RobotConfigurationData> publisher;

   public FrameRealtimeROS2PublisherSubscriberTest()
   {
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "frameTest");
      try
      {
         String topic = "FrameData";
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
            RobotConfigurationData RobotConfigurationData = new RobotConfigurationData();
            while (true)
            {
               boolean got = queuedSubscription.flushAndGetLatest(RobotConfigurationData);
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

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new FrameRealtimeROS2PublisherSubscriberTest();
   }
}
