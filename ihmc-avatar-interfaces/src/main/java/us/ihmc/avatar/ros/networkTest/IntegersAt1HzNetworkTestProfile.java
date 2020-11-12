package us.ihmc.avatar.ros.networkTest;

import org.apache.commons.lang3.mutable.MutableInt;
import std_msgs.msg.dds.Int64;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class IntegersAt1HzNetworkTestProfile extends ROS2NetworkTestProfile
{
   private final ROS2Topic<Int64> topic = ROS2Tools.IHMC_ROOT.withModule("tester").withType(Int64.class).withSuffix("ints1hz");
   private final MutableInt number = new MutableInt();
   private final IHMCROS2Publisher<Int64> publisher;

   private final YoRegistry yoRegistry = new YoRegistry(localHostname + getClass().getSimpleName());
   private ROS2NetworkTestMachine machine;

   public IntegersAt1HzNetworkTestProfile()
   {
      for (ROS2NetworkTestMachine machine : getMachines())
      {
         if (machine.getHostname().equals(localHostname))
         {
            this.machine = machine;
         }
      }

      if (machine == null)
      {
         LogTools.info("This machine is excluded from this test.");
      }
      else
      {
         LogTools.info("Profile active.");
      }

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "profile");

      publisher = ROS2Tools.createPublisher(ros2Node, topic);
   }

   @Override
   public List<ROS2NetworkTestMachine> getMachines()
   {
      ArrayList<ROS2NetworkTestMachine> machines = new ArrayList<>();
      machines.add(new ROS2NetworkTestMachine(localHostname, "robotlab", "unused"));
      machines.add(new ROS2NetworkTestMachine("cpu0", "shadylady", "/usr/local/bin/mission_control/bin"));
      machines.add(new ROS2NetworkTestMachine("cpu1", "shadylady", "/usr/local/bin/mission_control/bin"));
      machines.add(new ROS2NetworkTestMachine("cpu4", "shadylady", "/usr/local/bin/mission_control/bin"));
      return machines;
   }

   @Override
   public void runExperiment()
   {
      new PausablePeriodicThread(getClass().getSimpleName(), 1.0, () ->
      {
         Int64 message = new Int64();
         message.setData(number.getAndIncrement());
         publisher.publish(message);
      }).start();

      // TODO
      // start subscribing and logging expected vs actual


      ThreadTools.sleepForever();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return yoRegistry;
   }
}
