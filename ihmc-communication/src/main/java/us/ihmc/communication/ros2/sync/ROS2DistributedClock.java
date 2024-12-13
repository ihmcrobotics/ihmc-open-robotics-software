package us.ihmc.communication.ros2.sync;

import ihmc_common_msgs.msg.dds.DistributedClockMessage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.common.Time;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.net.InetAddress;
import java.time.Instant;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Assumes symmetric network delay.
 */
public class ROS2DistributedClock
{
   private static final AtomicLong INDEX = new AtomicLong();
   private static final ROS2Topic<DistributedClockMessage> TOPIC = ROS2Tools.IHMC_ROOT.withModule("distributed_clock").withType(DistributedClockMessage.class);

   private final String ourName;
   private final HashMap<String, ROS2DistributedClockPeer> peers = new HashMap<>();

   private final SampleInfo receivedSampleInfo = new SampleInfo();
   private final DistributedClockMessage receivedClockMessage = new DistributedClockMessage();

   private final RepeatingTaskThread requestThread = new RepeatingTaskThread(getClass().getSimpleName(),
                                                                             this::requestThread,
                                                                             DefaultExceptionHandler.MESSAGE_AND_STACKTRACE)
                                                         .setFrequencyLimit(1.0);

   public ROS2DistributedClock(ROS2Node ros2Node)
   {
      AtomicReference<String> hostname = new AtomicReference<>("");
      ExceptionTools.handle(() -> hostname.set("_" + InetAddress.getLocalHost().getHostName()), DefaultExceptionHandler.PROCEED_SILENTLY);


      ourName = "%s%s_%d_%d".formatted(ros2Node.getName(), hostname.get(), ProcessHandle.current().pid(), INDEX.getAndIncrement());


      ros2Node.createSubscription(TOPIC, subscriber ->
      {
         Instant now = Instant.now();

         long epochSecond = now.getEpochSecond();
         int nano = now.getNano();

         subscriber.takeNextData(receivedClockMessage, receivedSampleInfo);

         Time sourceTimestamp = receivedSampleInfo.getSourceTimestamp();
         long nanoseconds = sourceTimestamp.getNanoseconds();
         int seconds = sourceTimestamp.getSeconds();


      });



      requestThread.start();
   }

   private void requestThread()
   {

   }
}
