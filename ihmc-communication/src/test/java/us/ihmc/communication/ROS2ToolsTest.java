package us.ihmc.communication;

import static org.junit.jupiter.api.Assertions.assertEquals;

import perception_msgs.msg.dds.REAStateRequestMessage;
import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import std_msgs.msg.dds.Float64;
import std_msgs.msg.dds.Int64;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2Topic;
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
      assertEquals("/ihmc/rea_state_request", ROS2Tools.IHMC_ROOT.withTypeName(REAStateRequestMessage.class).toString());
      assertEquals("/ihmc/atlas/rea_state_request", ROS2Tools.IHMC_ROOT.withTypeName(REAStateRequestMessage.class).withRobot("atlas").toString());
      assertEquals("/ihmc/atlas/rea/input/rea_state_request",
                   ROS2Tools.IHMC_ROOT.withTypeName(REAStateRequestMessage.class).withRobot("atlas").withModule("rea").withInput().toString());

      ROS2Topic<?> defaultTopicName = ROS2Tools.IHMC_ROOT;
      assertEquals("/ihmc/rea_state_request", defaultTopicName.withTypeName(REAStateRequestMessage.class).toString());

      ROS2Topic<?> defaultTopicNameWithRobot = ROS2Tools.IHMC_ROOT.withRobot("atlas");
      assertEquals("/ihmc/atlas/rea_state_request", defaultTopicNameWithRobot.withTypeName(REAStateRequestMessage.class).toString());

      ROS2Topic<?> defaultTopicName3 = ROS2Tools.IHMC_ROOT.withRobot("atlas").withModule("rea").withOutput();
      assertEquals("/ihmc/atlas/rea/output/rea_state_request", defaultTopicName3.withTypeName(REAStateRequestMessage.class).toString());

      assertEquals("/ihmc/atlas/toolbox/teleop/step_teleop/output", ToolboxAPIs.STEP_TELEOP_TOOLBOX.withRobot("atlas").withOutput().toString());
   }

   @Disabled
   @Test
   public void testPublishingWithinCallbackThrowsException()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName());

      ROS2Helper helper = new ROS2Helper(ros2Node);

      ROS2Topic<Int64> intTopic = ROS2Tools.IHMC_ROOT.withTypeName(Int64.class);
      ROS2Topic<Float64> doubleTopic = ROS2Tools.IHMC_ROOT.withTypeName(Float64.class);

      MutableInt intCount = new MutableInt();
      MutableInt doubleCount = new MutableInt();
      helper.subscribeViaCallback(intTopic, number ->
      {
         LogTools.info("Received int #{}: {}", intCount.getAndIncrement(), number);
         Float64 num = new Float64();
         num.setData(System.nanoTime() / 2.0);
         LogTools.info("Publishing: {}", num.getData());
         helper.publish(doubleTopic, num);
      });
      helper.subscribeViaCallback(doubleTopic, number ->
      {
         LogTools.info("Received double #{}: {}", doubleCount.getAndIncrement(), number);
      });

      new ExceptionHandlingThreadScheduler(getClass().getSimpleName()).schedule(() ->
                                                                                {
                                                                                   Int64 num = new Int64();
                                                                                   num.setData(System.nanoTime());
                                                                                   LogTools.info("Publishing: {}", num.getData());
                                                                                   helper.publish(intTopic, num);
                                                                                }, 1.0);

      ThreadTools.sleepForever();
   }

   public void testROS2Communication()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName());

      ROS2PublisherBasics<Int64> intPublisher = ros2Node.createPublisher(ROS2Tools.IHMC_ROOT.withTypeName(Int64.class));

      MutableInt count = new MutableInt();
      new ROS2Callback<>(ros2Node, Int64.class, ROS2Tools.IHMC_ROOT, message ->
      {
         LogTools.info("Received int #{}: {}", count.getAndIncrement(), message);
      });

      new ExceptionHandlingThreadScheduler(getClass().getSimpleName()).schedule(() ->
      {
         Int64 num = new Int64();
         num.setData(System.nanoTime());
         LogTools.info("Publishing: {}", num.getData());
         intPublisher.publish(num);
      }, 1.0);

      ThreadTools.sleepForever();
   }
}
