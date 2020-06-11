package us.ihmc.humanoidBehaviors.tools;

import org.junit.jupiter.api.Test;
import std_msgs.msg.dds.String;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.tools.ros2.ManagedROS2Node;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.ROS2TopicNameTools;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2PublisherBasics;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

public class ManagedROS2NodeTest
{
   private String newMessage(java.lang.String string)
   {
      String message = new String();
      message.setData(string);
      return message;
   }

   private TopicDataType<String> newTopicDataType()
   {
      return ROS2TopicNameTools.newMessageTopicDataTypeInstance(String.class);
   }

   @Test
   public void test() throws IOException
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(DomainFactory.PubSubImplementation.INTRAPROCESS, "TestNode");
      ManagedROS2Node managedROS2Node = new ManagedROS2Node(ros2Node);

      TypedNotification<java.lang.String> topicOneNotification = new TypedNotification<>();
      managedROS2Node.createSubscription(newTopicDataType(), message -> topicOneNotification.set(message.takeNextData().getDataAsString()), "/topicOne");

      Ros2PublisherBasics<String> topicOnePublisher = managedROS2Node.createPublisher(newTopicDataType(), "/topicOne");

      topicOnePublisher.publish(newMessage("1"));

      Stopwatch stopwatch = new Stopwatch().start();
      assertEquals("1", topicOneNotification.blockingPoll());
      LogTools.info("Took {}", stopwatch.totalElapsed());

      managedROS2Node.setEnabled(false);

      topicOnePublisher.publish(newMessage("2"));
      ThreadTools.sleep(50);
      assertFalse(topicOneNotification.poll());

      managedROS2Node.setEnabled(true);

      topicOnePublisher.publish(newMessage("3"));
      ThreadTools.sleep(50);
      assertTrue(topicOneNotification.poll());
      assertEquals("3", topicOneNotification.read());

      managedROS2Node.setEnabled(false);

      Ros2PublisherBasics<String> topicTwoPublisher = managedROS2Node.createPublisher(newTopicDataType(), "/topicTwo");
      TypedNotification<java.lang.String> topicTwoNotification = new TypedNotification<>();
      managedROS2Node.createSubscription(newTopicDataType(), message -> topicTwoNotification.set(message.takeNextData().getDataAsString()), "/topicTwo");

      topicTwoPublisher.publish(newMessage("5"));
      ThreadTools.sleep(50);
      assertFalse(topicTwoNotification.poll());

      managedROS2Node.setEnabled(true);

      topicTwoPublisher.publish(newMessage("6"));
      ThreadTools.sleep(50);
      assertTrue(topicTwoNotification.poll());
      assertEquals("6", topicTwoNotification.read());
   }
}
