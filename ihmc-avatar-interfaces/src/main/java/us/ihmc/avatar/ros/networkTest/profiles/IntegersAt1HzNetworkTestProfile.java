package us.ihmc.avatar.ros.networkTest.profiles;

import org.apache.commons.lang3.mutable.MutableInt;
import std_msgs.msg.dds.Int64;
import us.ihmc.avatar.ros.networkTest.ROS2NetworkTestMachine;
import us.ihmc.avatar.ros.networkTest.ROS2NetworkTestProfile;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.avatar.ros.networkTest.ROS2NetworkTestMachine.*;

public class IntegersAt1HzNetworkTestProfile extends ROS2NetworkTestProfile
{
   private static final ROS2Topic<Int64> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("ints1hz").withType(Int64.class);
   private static final ROS2Topic<Int64> TO_OCU = BASE_TOPIC.withSuffix("toocu");
   private static final ROS2Topic<Int64> TO_CPU1 = BASE_TOPIC.withSuffix("tocpu1");
   private final MutableInt number = new MutableInt();
   private final IHMCROS2Publisher<Int64> publisher;

   private final YoRegistry yoRegistry = new YoRegistry(getMachineName() + getClass().getSimpleName());
   private final YoLong messagesSent = new YoLong(getMachineName() + "Sent", yoRegistry);
   private final YoLong messagesReceived = new YoLong(getMachineName() + "Received", yoRegistry);

   private PausablePeriodicThread experimentThread;

   public IntegersAt1HzNetworkTestProfile()
   {
      LogTools.info("Running on {}", getMachineName());

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "profile");

      ROS2Topic<Int64> publisherTopic = null;
      ROS2Topic<Int64> subscriberTopic = null;
      if (getLocalMachine() == CPU1)
      {
         publisherTopic = TO_OCU;
         subscriberTopic = TO_CPU1;
      }
      else if (getLocalMachine() == CPU0)
      {
         publisherTopic = TO_OCU;
      }
      else if (getLocalMachine() == CPU4)
      {
         publisherTopic = TO_OCU;
      }
      else// if (localHostname.equals("atlas-ocu"))
      {
         publisherTopic = TO_CPU1;
         subscriberTopic = TO_OCU;
      }

      publisher = ROS2Tools.createPublisher(ros2Node, publisherTopic);
      if (subscriberTopic != null)
      {
         new IHMCROS2Callback<>(ros2Node, subscriberTopic, message ->
         {
            messagesReceived.add(1);
         });
      }
   }

   @Override
   public List<ROS2NetworkTestMachine> getMachines()
   {
      ArrayList<ROS2NetworkTestMachine> machines = new ArrayList<>();
      machines.add(OCU);
      machines.add(CPU0);
      machines.add(CPU1);
      machines.add(CPU4);
      return machines;
   }

   @Override
   public void runExperiment()
   {
      experimentThread = new PausablePeriodicThread(getClass().getSimpleName(), 1.0, () ->
      {
         Int64 message = new Int64();
         message.setData(number.getAndIncrement());
         LogTools.info("Publishing {}", number.getValue());
         messagesSent.add(1);
         publisher.publish(message);
      });
      experimentThread.start();

      // TODO
      // start subscribing and logging expected vs actual

      ThreadTools.sleepSeconds(10.0);
   }

   @Override
   public List<String[]> getGraphsToSetup()
   {
      ArrayList<String[]> graphsToSetup = new ArrayList<>();
      graphsToSetup.add(new String[] {"cpu0Sent", "cpu1Sent", "cpu4Sent", "ocuReceived"});
      graphsToSetup.add(new String[] {"ocuSent", "cpu1Received"});
      return graphsToSetup;
   }

   @Override
   public void destroy()
   {
      if (experimentThread != null)
         experimentThread.destroy();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return yoRegistry;
   }
}
