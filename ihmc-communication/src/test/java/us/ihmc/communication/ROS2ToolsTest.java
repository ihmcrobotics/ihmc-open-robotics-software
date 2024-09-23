package us.ihmc.communication;

import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.REAStateRequestMessage;
import std_msgs.msg.dds.Float64;
import std_msgs.msg.dds.Int64;
import std_msgs.msg.dds.String;
import test_msgs.msg.dds.LongString;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Enumeration;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.*;

class ROS2ToolsTest
{
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

   @Disabled
   @Test
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

   @Disabled
   @Test
   public void testStringCommunication()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName());

      ROS2PublisherBasics<String> stringPublisher = ros2Node.createPublisher(ROS2Tools.IHMC_ROOT.withType(String.class));

      MutableInt count = new MutableInt();
      new ROS2Callback<>(ros2Node, String.class, ROS2Tools.IHMC_ROOT, message ->
      {
         LogTools.info("Received int #{}: {}", count.getAndIncrement(), message);
      });

      new ExceptionHandlingThreadScheduler(getClass().getSimpleName()).schedule(() ->
      {
         String message = new String();
         StringBuilder builder = new StringBuilder();
         for (int i = 0; i < 100; i++)
            builder.append(i);
         message.setData(builder.toString());
         LogTools.info("Publishing: {}", message.getData());
         stringPublisher.publish(message);
      }, 1.0);

      ThreadTools.sleepForever();
   }

   @Disabled
   @Test
   public void testLongStringCommunication()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName());

      ROS2PublisherBasics<LongString> stringPublisher = ros2Node.createPublisher(ROS2Tools.IHMC_ROOT.withType(LongString.class));

      MutableInt count = new MutableInt();
      new ROS2Callback<>(ros2Node, LongString.class, ROS2Tools.IHMC_ROOT, message ->
      {
         LogTools.info("Received int #{}: {}", count.getAndIncrement(), MessageTools.unpackLongStringFromByteSequence(message.getLongString()));
      });

      new ExceptionHandlingThreadScheduler(getClass().getSimpleName()).schedule(() ->
      {
         LongString message = new LongString();
         StringBuilder builder = new StringBuilder();
         for (int i = 0; i < 500; i++)
            builder.append(i);
         java.lang.String string = builder.toString();
         MessageTools.packLongStringToByteSequence(string, message.getLongString());
         LogTools.info("Publishing: {}", string);
         stringPublisher.publish(message);
      }, 1.0);

      ThreadTools.sleepForever();
   }

   @Test
   public void testLoopbackROS2Node() throws InterruptedException, IOException
   {
      AtomicBoolean messageReceived = new AtomicBoolean(false);
      AtomicBoolean failed = new AtomicBoolean(false);
      StringBuilder stringToSend = new StringBuilder("Hello World!");

      ROS2Node loopbackNode = ROS2Tools.createLoopbackROS2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName() + "LoopbackNode");
      ROS2Subscription<String> loopbackSubscriber = loopbackNode.createSubscription(ROS2Tools.IHMC_ROOT.withType(String.class), subscriber ->
      {
         String message = subscriber.takeNextData();
         messageReceived.set(true);
         synchronized (messageReceived)
         {
            messageReceived.notify();
         }
         assertEquals(stringToSend.toString(), message.getDataAsString());
         if (!stringToSend.toString().equals(message.getDataAsString()))
            failed.set(true);
      });

      InetAddress outsiderAddress = getPhysicalAddress();
      LogTools.info("Outsider node on {}", outsiderAddress);
      ROS2Node outsiderNode = new ROS2Node(PubSubImplementation.FAST_RTPS,
                                           getClass().getSimpleName(),
                                           new RTPSCommunicationFactory().getDomainId(),
                                           outsiderAddress);
      ROS2Subscription<String> outsideSubscriber = outsiderNode.createSubscription(ROS2Tools.IHMC_ROOT.withType(String.class), subscriber ->
      {
         LogTools.error("Outsider node should NOT receive any messages");
         failed.set(true);
      });

      ROS2PublisherBasics<String> stringPublisher = loopbackNode.createPublisher(ROS2Tools.IHMC_ROOT.withType(String.class));
      String messageToSend = new String();
      messageToSend.setData(stringToSend.toString());
      stringPublisher.publish(messageToSend);

      synchronized (messageReceived)
      {
         messageReceived.wait(1000);
      }
      assertTrue(messageReceived.get());
      assertFalse(failed.get());

      loopbackSubscriber.remove();
      outsideSubscriber.remove();
   }

   private InetAddress getPhysicalAddress() throws IOException
   {
      Enumeration<NetworkInterface> interfaces = NetworkInterface.getNetworkInterfaces();
      while (interfaces.hasMoreElements())
      {
         NetworkInterface intrface = interfaces.nextElement();
         if (intrface.isUp() && !intrface.isLoopback() && !intrface.isVirtual())
         {
            Enumeration<InetAddress> addresses = intrface.getInetAddresses();
            while (addresses.hasMoreElements())
            {
               InetAddress address = addresses.nextElement();
               if (address instanceof Inet4Address && address.isReachable(1000))
                  return address;
            }
         }
      }

      return null;
   }
}
