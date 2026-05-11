package us.ihmc.communication.ros2;

import controller_msgs.RobotConfigurationData;
import org.junit.jupiter.api.Disabled;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;

@Disabled // TODO: jros2 migration - QueuedROS2Subscription not ported yet
public class FrameRealtimeROS2PublisherSubscriberTest
{
   private AsyncROS2Node realtimeROS2Node;
   private ROS2Publisher<RobotConfigurationData> publisher;

   public FrameRealtimeROS2PublisherSubscriberTest()
   {
      realtimeROS2Node = new AsyncROS2Node("frameTest");
      ROS2Topic<RobotConfigurationData> topic = ROS2Tools.IHMC_ROOT.appendedWith("FrameData").withType(RobotConfigurationData.class);
      LogTools.info("Publishing to {}", topic);
      publisher = realtimeROS2Node.createPublisher(topic, ROS2QoSProfile.BEST_EFFORT);

      ROS2Subscription<RobotConfigurationData> subscriber = realtimeROS2Node.createSubscription(topic, reader ->
      {
         RobotConfigurationData message = reader.read();
         LogTools.info("Got from callback");
      }, ROS2QoSProfile.BEST_EFFORT);

      // QueuedROS2Subscription not available in jros2 yet - test disabled

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new FrameRealtimeROS2PublisherSubscriberTest();
   }
}
