package us.ihmc.communication;

import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import sensor_msgs.CameraInfo;
import std_msgs.Float64;
import std_msgs.Int64;
import test_msgs.LongString;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.jros2.ROS2Node;
// import us.ihmc.ros2.ROS2NodeBuilder.SpecialTransportMode; // TODO: jros2 migration - ROS2NodeBuilder not ported
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.communication.HumanoidROS2Topic;
import us.ihmc.jros2.ROS2Topic;
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
      assertEquals("/ihmc/camera_info", ROS2Tools.IHMC_ROOT.withTypeName(CameraInfo.class).toString());
      assertEquals("/ihmc/atlas/camera_info", ROS2Tools.IHMC_ROOT.withTypeName(CameraInfo.class).withRobot("atlas").toString());
      assertEquals("/ihmc/atlas/rea/input/camera_info",
                   ROS2Tools.IHMC_ROOT.withTypeName(CameraInfo.class).withRobot("atlas").withModule("rea").withInput().toString());

      HumanoidROS2Topic<?> defaultTopicName = ROS2Tools.IHMC_ROOT;
      assertEquals("/ihmc/camera_info", defaultTopicName.withTypeName(CameraInfo.class).toString());

      HumanoidROS2Topic<?> defaultTopicNameWithRobot = ROS2Tools.IHMC_ROOT.withRobot("atlas");
      assertEquals("/ihmc/atlas/camera_info", defaultTopicNameWithRobot.withTypeName(CameraInfo.class).toString());

      HumanoidROS2Topic<?> defaultTopicName3 = ROS2Tools.IHMC_ROOT.withRobot("atlas").withModule("rea").withOutput();
      assertEquals("/ihmc/atlas/rea/output/camera_info", defaultTopicName3.withTypeName(CameraInfo.class).toString());

      assertEquals("/ihmc/atlas/toolbox/teleop/step_teleop/output", ToolboxAPIs.STEP_TELEOP_TOOLBOX.withRobot("atlas").withOutput().toString());
      assertEquals("/ihmc/atlas/controller", ControllerAPI.getBaseTopic("controller", "atlas").toString());
   }

   @Disabled
   @Test
   public void testPublishingWithinCallbackThrowsException()
   {
      ROS2Node ros2Node = new ROS2Node(getClass().getSimpleName());

      ROS2Helper helper = new ROS2Helper(ros2Node);

      ROS2Topic<Int64> intTopic = ROS2Tools.IHMC_ROOT.withType(Int64.class);
      ROS2Topic<Float64> doubleTopic = ROS2Tools.IHMC_ROOT.withType(Float64.class);

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
      ROS2Node ros2Node = new ROS2Node(getClass().getSimpleName());

      ROS2Publisher<Int64> intPublisher = ros2Node.createPublisher(ROS2Tools.IHMC_ROOT.withType(Int64.class));

      MutableInt count = new MutableInt();
      ros2Node.createSubscription(ROS2Tools.IHMC_ROOT.withType(Int64.class), reader ->
      {
         Int64 message = reader.read();
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
      ROS2Node ros2Node = new ROS2Node(getClass().getSimpleName());

      ROS2Publisher<std_msgs.String_> stringPublisher = ros2Node.createPublisher(ROS2Tools.IHMC_ROOT.withType(std_msgs.String_.class));

      MutableInt count = new MutableInt();
      ros2Node.createSubscription(ROS2Tools.IHMC_ROOT.withType(std_msgs.String_.class), reader ->
      {
         std_msgs.String_ message = reader.read();
         LogTools.info("Received int #{}: {}", count.getAndIncrement(), message);
      });

      new ExceptionHandlingThreadScheduler(getClass().getSimpleName()).schedule(() ->
      {
         std_msgs.String_ message = new std_msgs.String_();
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
      ROS2Node ros2Node = new ROS2Node(getClass().getSimpleName());

      ROS2Publisher<LongString> stringPublisher = ros2Node.createPublisher(ROS2Tools.IHMC_ROOT.withType(LongString.class));

      MutableInt count = new MutableInt();
      ros2Node.createSubscription(ROS2Tools.IHMC_ROOT.withType(LongString.class), reader ->
      {
         LongString message = reader.read();
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

   @Disabled // TODO: jros2 migration - ROS2NodeBuilder not ported yet
   @Test
   public void testSharedMemoryROS2Node() throws InterruptedException
   {
      AtomicBoolean messageReceived = new AtomicBoolean(false);
      AtomicBoolean failed = new AtomicBoolean(false);
      StringBuilder stringToSend = new StringBuilder("Hello World!");

      // ROS2Node sharedMemoryNode = new ROS2NodeBuilder().specialTransportMode(SpecialTransportMode.SHARED_MEMORY_ONLY).build(getClass().getSimpleName() + "SharedMemoryNode");
      ROS2Node sharedMemoryNode = new ROS2Node(getClass().getSimpleName() + "SharedMemoryNode");
      ROS2Subscription<std_msgs.String_> sharedMemorySubscriber = sharedMemoryNode.createSubscription(ROS2Tools.IHMC_ROOT.withType(std_msgs.String_.class), reader ->
      {
         std_msgs.String_ message = reader.read();
         messageReceived.set(true);
         synchronized (messageReceived)
         {
            messageReceived.notify();
         }
         assertEquals(stringToSend.toString(), message.getDataAsString());
         if (!stringToSend.toString().equals(message.getDataAsString()))
            failed.set(true);
      });

      ROS2Publisher<std_msgs.String_> publisher = sharedMemoryNode.createPublisher(ROS2Tools.IHMC_ROOT.withType(std_msgs.String_.class));
      std_msgs.String_ messageToSend = new std_msgs.String_();
      messageToSend.setData(stringToSend.toString());
      publisher.publish(messageToSend);

      synchronized (messageReceived)
      {
         messageReceived.wait(1000);
      }
      assertTrue(messageReceived.get());
      assertFalse(failed.get());

      // sharedMemorySubscriber.close(); // TODO: jros2 - no close() method on ROS2Subscription anymore
   }

   @Disabled // TODO: jros2 migration - ROS2NodeBuilder not ported yet
   @Test
   public void testLoopbackROS2Node() throws InterruptedException, IOException
   {
      AtomicBoolean messageReceived = new AtomicBoolean(false);
      AtomicBoolean failed = new AtomicBoolean(false);
      StringBuilder stringToSend = new StringBuilder("Hello World!");

      int domainId = 111;

      // ROS2Node loopbackNode = new ROS2NodeBuilder().domainId(domainId)
      //                                              .specialTransportMode(SpecialTransportMode.UDPV4_LOOPBACK_ADDRESS_ONLY)
      //                                              .build(getClass().getSimpleName() + "LoopbackNode");
      ROS2Node loopbackNode = new ROS2Node(getClass().getSimpleName() + "LoopbackNode", domainId);
      ROS2Subscription<std_msgs.String_> loopbackSubscriber = loopbackNode.createSubscription(ROS2Tools.IHMC_ROOT.withType(std_msgs.String_.class), reader -> {
         std_msgs.String_ message = reader.read();
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
      // ROS2Node outsiderNode = new ROS2NodeBuilder().domainId(domainId).addressRestriction(outsiderAddress).build(getClass().getSimpleName());
      ROS2Node outsiderNode = new ROS2Node(getClass().getSimpleName(), domainId);
      ROS2Subscription<std_msgs.String_> outsideSubscriber = outsiderNode.createSubscription(ROS2Tools.IHMC_ROOT.withType(std_msgs.String_.class), reader ->
      {
         LogTools.error("Outsider node should NOT receive any messages");
         failed.set(true);
      });

      ROS2Publisher<std_msgs.String_> stringPublisher = loopbackNode.createPublisher(ROS2Tools.IHMC_ROOT.withType(std_msgs.String_.class));
      std_msgs.String_ messageToSend = new std_msgs.String_();
      messageToSend.setData(stringToSend.toString());
      stringPublisher.publish(messageToSend);

      synchronized (messageReceived)
      {
         messageReceived.wait(1000);
      }
      assertTrue(messageReceived.get());
      assertFalse(failed.get());

      // loopbackSubscriber.close(); // TODO: jros2 - no close() method on ROS2Subscription anymore
      // outsideSubscriber.close(); // TODO: jros2 - no close() method on ROS2Subscription anymore
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
