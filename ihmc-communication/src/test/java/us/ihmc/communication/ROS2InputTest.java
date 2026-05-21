package us.ihmc.communication;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import std_msgs.Empty;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;

public class ROS2InputTest
{
   @Test
   public void test()
   {
      ROS2Node ros2Node = new ROS2Node("test_input");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      ROS2Topic<Empty> inputTestTopic = ROS2Tools.IHMC_ROOT.withSuffix("input_test_topic").withType(Empty.class);

      TypedNotification<Empty> subscription = ros2Helper.subscribeViaTypedNotification(inputTestTopic);

      // Testing that the subscription doesn't return a notification immediately
      Assertions.assertFalse(subscription.poll());

      // Publish to the topic
      ros2Helper.publish(inputTestTopic, new Empty());

      // Create a subscriber immediately after publishing
      TypedNotification<Empty> subscription3 = ros2Helper.subscribeViaTypedNotification(inputTestTopic);
      Assertions.assertFalse(subscription3.poll());

      // Wait for the original subscriber to get it
      subscription.blockingPeek();

      // Make sure the notification returns true once only
      Assertions.assertTrue(subscription.poll());
      Assertions.assertFalse(subscription.poll());
      Assertions.assertFalse(subscription.poll());


      // It appears that the subscriber created after publish will never get the message
      ThreadTools.sleepSeconds(0.05);
      Assertions.assertFalse(subscription3.poll());

      // Create a subscriber later and make sure it doesn't get a notification
      TypedNotification<Empty> subscription2 = ros2Helper.subscribeViaTypedNotification(inputTestTopic);
      Assertions.assertFalse(subscription2.poll());

      ros2Node.close();
   }
}
