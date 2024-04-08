package us.ihmc.communication;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import std_msgs.msg.dds.Empty;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

public class ROS2InputTest
{
   @Test
   public void test()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.INTRAPROCESS, "test_input");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      ROS2Topic<Empty> inputTestTopic = ROS2Tools.IHMC_ROOT.withSuffix("input_test_topic").withType(Empty.class);

      ROS2Input<?> subscription = ros2Helper.subscribe(inputTestTopic);

      // Testing that the subscription doesn't return a notification immediately
      Assertions.assertFalse(subscription.getMessageNotification().poll());

      // Publish to the topic
      ros2Helper.publish(inputTestTopic, new Empty());

      // Create a subscriber immediately after publishing
      ROS2Input<?> subscription3 = ros2Helper.subscribe(inputTestTopic);
      Assertions.assertFalse(subscription3.getMessageNotification().poll());

      // Wait for the original subscriber to get it
      subscription.getMessageNotification().blockingPeek();

      // Make sure the notification returns true once only
      Assertions.assertTrue(subscription.getMessageNotification().poll());
      Assertions.assertFalse(subscription.getMessageNotification().poll());
      Assertions.assertFalse(subscription.getMessageNotification().poll());


      // It appears that the subscriber created after publish will never get the message
      ThreadTools.sleepSeconds(0.05);
      Assertions.assertFalse(subscription3.getMessageNotification().poll());

      // Create a subscriber later and make sure it doesn't get a notification
      ROS2Input<?> subscription2 = ros2Helper.subscribe(inputTestTopic);
      Assertions.assertFalse(subscription2.getMessageNotification().poll());

      subscription3.destroy();
      subscription2.destroy();
      subscription.destroy();
      ros2Node.destroy();
   }
}
