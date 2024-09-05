package us.ihmc.perception.streaming;

import perception_msgs.msg.dds.SRTStreamRequest;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.net.InetSocketAddress;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class ROS2SRTVideoStreamer extends SRTVideoStreamer
{
   private final ROS2Node ros2Node;
   private final ExecutorService callerConnector = Executors.newCachedThreadPool();

   public ROS2SRTVideoStreamer(ROS2Topic<SRTStreamRequest> streamRequestTopic)
   {
      super();

      ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "srt_video_streamer");
      ros2Node.createSubscription(streamRequestTopic, this::processStreamRequests);
   }

   private void processStreamRequests(Subscriber<SRTStreamRequest> requestSubscriber)
   {
      SRTStreamRequest request = requestSubscriber.takeNextData();
      InetSocketAddress callerAddress = InetSocketAddress.createUnresolved(request.getReceiverAddressAsString(), request.getReceiverPort());
      if (request.getConnectionWanted())
         callerConnector.submit(() -> connectToCaller(callerAddress));
      else
         removeCaller(callerAddress);
   }

   @Override
   public void destroy()
   {
      super.destroy();

      callerConnector.shutdown();
      try
      {
         if (!callerConnector.awaitTermination(3, TimeUnit.SECONDS))
            callerConnector.shutdownNow();
      }
      catch (InterruptedException e)
      {
         callerConnector.shutdownNow();
      }
      ros2Node.destroy();
   }
}
