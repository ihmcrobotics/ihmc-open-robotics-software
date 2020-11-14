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
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.avatar.ros.networkTest.ROS2NetworkTestMachine.*;

public class IntegersAt100HzNetworkTestProfile extends ROS2NetworkTestProfile
{
   private static final ROS2Topic<Int64> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("ints100hz").withType(Int64.class);
   private static final ROS2Topic<Int64> TO_OCU = BASE_TOPIC.withSuffix("toocu");
   private final MutableInt number = new MutableInt();

   private final YoRegistry yoRegistry = new YoRegistry(getMachineName() + getClass().getSimpleName());
   private final YoLong messagesSent = new YoLong(getMachineName() + "Sent", yoRegistry);
   private final YoLong messagesReceived = new YoLong(getMachineName() + "Received", yoRegistry);

   private YoLong totalSent;

   private PausablePeriodicThread experimentThread;
   private final ROS2Node ros2Node;

   public IntegersAt100HzNetworkTestProfile()
   {
      LogTools.info("Running on {}", getMachineName());

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, getMachineName() + "ints100hz");

      if (getLocalMachine() == OCU)
      {
         totalSent = new YoLong("totalSent", yoRegistry);
      }
   }

   @Override
   public void updateDerivativeVariables(YoRegistry syncedRegistry)
   {
      YoLong cpu0Sent = (YoLong) syncedRegistry.findVariable("cpu0Sent");
      YoLong cpu1Sent = (YoLong) syncedRegistry.findVariable("cpu1Sent");
      YoLong cpu4Sent = (YoLong) syncedRegistry.findVariable("cpu4Sent");

//      System.out.println(" " + cpu0Sent + cpu1Sent + cpu4Sent + totalSent + " ");
      totalSent.set(cpu0Sent.getValue() + cpu1Sent.getValue() + cpu4Sent.getValue());
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
      if (getLocalMachine() == OCU)
      {
         new IHMCROS2Callback<>(ros2Node, TO_OCU, ROS2QosProfile.BEST_EFFORT(), message ->
         {
            messagesReceived.add(1);
         });
      }
      else
      {
         IHMCROS2Publisher<Int64> publisher = ROS2Tools.createPublisher(ros2Node, TO_OCU, ROS2QosProfile.BEST_EFFORT());
         experimentThread = new PausablePeriodicThread(getClass().getSimpleName(), UnitConversions.hertzToSeconds(100.0), () ->
         {
            if (messagesSent.getValue() < 1500)
            {
               Int64 message = new Int64();
               message.setData(number.getAndIncrement());
               messagesSent.add(1);
               publisher.publish(message);
            }
         });
         experimentThread.start();

      }

      ThreadTools.sleepSeconds(10.0);
   }

   @Override
   public List<String[]> getGraphsToSetup()
   {
      ArrayList<String[]> graphsToSetup = new ArrayList<>();
      graphsToSetup.add(new String[] {"cpu0Sent", "cpu1Sent", "cpu4Sent"});
      graphsToSetup.add(new String[] {"totalSent", "ocuReceived"});
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
