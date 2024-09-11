package us.ihmc.perception.streaming;

import perception_msgs.msg.dds.SRTStreamMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.perception.RawImage;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;

import java.net.InetSocketAddress;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import static us.ihmc.perception.streaming.StreamingTools.CONNECTION_TIMEOUT;
import static us.ihmc.perception.streaming.StreamingTools.STATUS_MESSAGE_UUID;

public class ROS2SRTVideoStreamer
{
   private final ROS2Node ros2Node;
   private final ROS2PublisherBasics<SRTStreamMessage> statusMessagePublisher;
   private final ROS2Input<SRTStreamMessage> requestMessageSubscription;

   /** The ACK message acknowledges a caller, and informs the caller of the camera intrinsics */
   private final SRTStreamMessage ackMessage;
   /** The status message is sent to all callers to inform them of the sensor pose */
   private final SRTStreamMessage statusMessage;

   private final SRTVideoStreamer videoStreamer;
   private final ExecutorService callerConnector = Executors.newCachedThreadPool();


   public ROS2SRTVideoStreamer(ROS2IOTopicPair<SRTStreamMessage> streamMessageTopicPair)
   {
      // Create and initialize the streamer
      videoStreamer = new SRTVideoStreamer();

      // Set up ROS2 subscriptions/publishers
      ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "srt_video_streamer");

      // Initialize the ACK message
      ackMessage = new SRTStreamMessage();

      // Initialize the status message
      statusMessage = new SRTStreamMessage();
      MessageTools.toMessage(STATUS_MESSAGE_UUID, statusMessage.getId());

      // Subscribe to stream request messages (the command topic)
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      requestMessageSubscription = ros2Helper.subscribe(streamMessageTopicPair.getCommandTopic(),
                                                        requestMessage -> !STATUS_MESSAGE_UUID.equals(MessageTools.toUUID(requestMessage.getId())));
      statusMessagePublisher = ros2Node.createPublisher(streamMessageTopicPair.getStatusTopic());
   }

   private void processStreamRequests(SRTStreamMessage request)
   {
      // Get the caller address
      InetSocketAddress callerAddress = InetSocketAddress.createUnresolved(request.getReceiverAddressAsString(), request.getReceiverPort());

      // See if caller wants to connect or disconnect
      if (request.getConnectionWanted())
      {
         // Start waiting for a connection
         callerConnector.submit(() -> videoStreamer.connectToCaller(callerAddress, CONNECTION_TIMEOUT));

         // Send ACK saying I'm ready
         ackMessage.getId().set(request.getId());
         statusMessagePublisher.publish(ackMessage);
      }
      else // Disconnecting: just remove the caller
         videoStreamer.removeCaller(callerAddress);
   }

   public void initialize(RawImage exampleImage, double cameraFPS, int avPixelFormat)
   {
      videoStreamer.initialize(exampleImage.getImageWidth(), exampleImage.getImageHeight(), cameraFPS, avPixelFormat);

      ackMessage.setImageWidth(exampleImage.getImageWidth());
      ackMessage.setImageHeight(exampleImage.getImageHeight());
      ackMessage.setFx(exampleImage.getFocalLengthX());
      ackMessage.setFy(exampleImage.getFocalLengthY());
      ackMessage.setCx(exampleImage.getPrincipalPointX());
      ackMessage.setCy(exampleImage.getPrincipalPointY());
      ackMessage.setDepthDiscretization(exampleImage.getDepthDiscretization());

      requestMessageSubscription.addCallback(this::processStreamRequests);
   }

   public void sendFrame(RawImage image)
   {
      if (image.get() == null)
         return;

      videoStreamer.sendFrame(image.getCpuImageMat());
      statusMessage.getPosition().set(image.getPosition());
      statusMessage.getOrientation().set(image.getOrientation());
      statusMessagePublisher.publish(statusMessage);

      image.release();
   }

   public void destroy()
   {
      videoStreamer.destroy();

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

      statusMessagePublisher.remove();
      requestMessageSubscription.destroy();
      ros2Node.destroy();
   }

   public int connectedCallerCount()
   {
      return videoStreamer.connectedCallerCount();
   }

   public boolean isInitialized()
   {
      return videoStreamer.isInitialized();
   }
}
