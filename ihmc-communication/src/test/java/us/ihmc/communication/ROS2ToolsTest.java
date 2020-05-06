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
import us.ihmc.ros2.ROS2TopicName;
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
      assertEquals("/ihmc/rea_state_request", ROS2Tools.IHMC_ROOT.type(REAStateRequestMessage.class).toString());
      assertEquals("/ihmc/atlas/rea_state_request", ROS2Tools.IHMC_ROOT.type(REAStateRequestMessage.class).robot("atlas").toString());
      assertEquals("/ihmc/atlas/rea/input/rea_state_request",
                   ROS2Tools.IHMC_ROOT.type(REAStateRequestMessage.class).robot("atlas").module("rea").qualifier(ROS2TopicQualifier.INPUT).toString());

      ROS2TopicName defaultTopicNameGenerator = ROS2Tools.IHMC_ROOT;
      assertEquals("/ihmc/rea_state_request", defaultTopicNameGenerator.type(REAStateRequestMessage.class).toString());

      ROS2TopicName defaultTopicNameGeneratorWithRobot = ROS2Tools.IHMC_ROOT.robot("atlas");
      assertEquals("/ihmc/atlas/rea_state_request", defaultTopicNameGeneratorWithRobot.type(REAStateRequestMessage.class).toString());

      ROS2TopicName defaultTopicNameGenerator3 = ROS2Tools.IHMC_ROOT.robot("atlas").module("rea").qualifier(ROS2TopicQualifier.OUTPUT);
      assertEquals("/ihmc/atlas/rea/output/rea_state_request", defaultTopicNameGenerator3.type(REAStateRequestMessage.class).toString());

      assertEquals("/ihmc/atlas/toolbox/teleop/step_teleop/output", ROS2Tools.STEP_TELEOP_TOOLBOX.robot("atlas").qualifier(ROS2TopicQualifier.OUTPUT).toString());
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
