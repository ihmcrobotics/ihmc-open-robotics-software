package us.ihmc.communication;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import std_msgs.Empty;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.HumanoidROS2Topic;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;

public class ROS2InputTest
{
   @Test
   public void test()
   {
      ROS2Node ros2Node = new ROS2Node("test_input");

      ROS2Topic<Empty> inputTestTopic = HumanoidROS2Topic.IHMC_ROOT.withSuffix("input_test_topic").withType(Empty.class);

      TypedNotification<Empty> subscription = new TypedNotification<>();
      ros2Node.createSubscription(inputTestTopic, reader ->
      {
         Empty message = reader.read();
         if (message != null)
            subscription.set(message);
      });

      // Testing that the subscription doesn't return a notification immediately
      Assertions.assertFalse(subscription.poll());

      // Publish to the topic
      ROS2Publisher<Empty> publisher = ros2Node.createPublisher(inputTestTopic);
      publisher.publish(new Empty());

      // Create a subscriber immediately after publishing
      TypedNotification<Empty> subscription3 = new TypedNotification<>();
      ros2Node.createSubscription(inputTestTopic, reader ->
      {
         Empty message = reader.read();
         if (message != null)
            subscription3.set(message);
      });
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
      TypedNotification<Empty> subscription2 = new TypedNotification<>();
      ros2Node.createSubscription(inputTestTopic, reader ->
      {
         Empty message = reader.read();
         if (message != null)
            subscription2.set(message);
      });
      Assertions.assertFalse(subscription2.poll());

      ros2Node.close();
   }
}
