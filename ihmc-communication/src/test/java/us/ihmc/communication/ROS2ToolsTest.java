package us.ihmc.communication;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import std_msgs.msg.dds.Int64;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

class ROS2ToolsTest
{
   public static void main(String[] args)
   {
      new ROS2ToolsTest().testROS2Communication();
   }

   public void testROS2Communication()
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName());

      IHMCROS2Publisher intPublisher = new IHMCROS2Publisher(ros2Node, Int64.class);

      new ROS2Callback<>(ros2Node, Int64.class, this::acceptMessage);

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
