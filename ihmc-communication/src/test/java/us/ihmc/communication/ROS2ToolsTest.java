package us.ihmc.communication;

import static org.junit.jupiter.api.Assertions.assertEquals;

import controller_msgs.msg.dds.REAStateRequestMessage;
import org.junit.jupiter.api.Test;

import std_msgs.msg.dds.Int64;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2MessageTopicNameGenerator;
import us.ihmc.ros2.ROS2TopicQualifier;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

class ROS2ToolsTest
{
   public static void main(String[] args)
   {
      new ROS2ToolsTest().testROS2Communication();
   }

   @Test
   public void testTopicNameStuff()
   {
      assertEquals("/ihmc/rea_state_request", ROS2Tools.generateDefaultTopicName(REAStateRequestMessage.class));
      assertEquals("/ihmc/atlas/rea_state_request", ROS2Tools.generateDefaultTopicName(REAStateRequestMessage.class, "atlas"));
      assertEquals("/ihmc/atlas/rea/input/rea_state_request",
                   ROS2Tools.generateDefaultTopicName(REAStateRequestMessage.class, "atlas", "rea", ROS2TopicQualifier.INPUT));

      ROS2MessageTopicNameGenerator defaultTopicNameGenerator = ROS2Tools.getDefaultTopicNameGenerator();
      assertEquals("/ihmc/rea_state_request", defaultTopicNameGenerator.generateTopicName(REAStateRequestMessage.class));

      ROS2MessageTopicNameGenerator defaultTopicNameGeneratorWithRobot = ROS2Tools.getDefaultTopicNameGenerator("atlas");
      assertEquals("/ihmc/atlas/rea_state_request", defaultTopicNameGeneratorWithRobot.generateTopicName(REAStateRequestMessage.class));

      ROS2MessageTopicNameGenerator defaultTopicNameGenerator3 = ROS2Tools.getTopicNameGenerator("atlas", "rea", ROS2TopicQualifier.OUTPUT);
      assertEquals("/ihmc/atlas/rea/output/rea_state_request", defaultTopicNameGenerator3.generateTopicName(REAStateRequestMessage.class));
   }

   public void testROS2Communication()
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName());

      IHMCROS2Publisher intPublisher = new IHMCROS2Publisher(ros2Node, Int64.class, ROS2Tools.IHMC_ROOT);

      new ROS2Callback<>(ros2Node, Int64.class, ROS2Tools.IHMC_ROOT, this::acceptMessage);

      new ExceptionHandlingThreadScheduler(getClass().getSimpleName()).schedule(() ->
      {
         Int64 num = new Int64();
         num.setData(System.nanoTime());
         LogTools.info("Publishing: {}", num.getData());
         intPublisher.publish(num);
      }, 1.0);

      ExceptionTools.handle(() -> Thread.currentThread().join(), DefaultExceptionHandler.PRINT_STACKTRACE);
   }

   private void acceptMessage(Int64 message)
   {
      LogTools.info("Received int: {}", message);
   }
}
