package us.ihmc.avatar.ros.networkTest;

import org.apache.commons.lang3.mutable.MutableInt;
import std_msgs.msg.dds.Int64;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.ArrayList;
import java.util.List;

public class IntegersAt1HzNetworkTestProfile extends ROS2NetworkTestProfile
{
   private final ROS2Topic<Int64> topic = ROS2Tools.IHMC_ROOT.withModule("tester").withType(Int64.class).withSuffix("ints1hz");
   private final MutableInt number = new MutableInt();
   private final IHMCROS2Publisher<Int64> publisher;

   public IntegersAt1HzNetworkTestProfile()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "profile");

      publisher = ROS2Tools.createPublisher(ros2Node, topic);
   }

   @Override
   public List<String> getHostnames()
   {
      ArrayList<String> hostnames = new ArrayList<>();
      hostnames.add(localHostName);
      hostnames.add("cpu0");
      hostnames.add("cpu1");
      hostnames.add("cpu4");
      return hostnames;
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
}
