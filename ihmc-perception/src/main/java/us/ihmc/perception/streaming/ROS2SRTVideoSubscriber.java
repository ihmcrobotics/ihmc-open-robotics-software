package us.ihmc.perception.streaming;

import perception_msgs.msg.dds.SRTStreamRequest;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.ros2.ROS2Topic;

import java.net.InetSocketAddress;

public class ROS2SRTVideoSubscriber extends SRTVideoSubscriber
{
   private static final double CONNECTION_TIMEOUT = 2.0;

   private final ROS2PublishSubscribeAPI ros2;
   private final ROS2Topic<SRTStreamRequest> streamRequestTopic;
   private final SRTStreamRequest requestMessage;

   public ROS2SRTVideoSubscriber(ROS2PublishSubscribeAPI ros2, ROS2Topic<SRTStreamRequest> streamRequestTopic, InetSocketAddress inputAddress, int outputAVPixelFormat)
   {
      super(inputAddress, outputAVPixelFormat);

      this.ros2 = ros2;
      this.streamRequestTopic = streamRequestTopic;

      requestMessage = new SRTStreamRequest();
      requestMessage.setReceiverAddress(inputAddress.getHostString());
      requestMessage.setReceiverPort(inputAddress.getPort());
   }

   public void subscribe()
   {
      // Keep requesting a connection until connected.
      while (!isConnected())
      {
         requestMessage.setConnectionWanted(true);
         ros2.publish(streamRequestTopic, requestMessage);
         waitForConnection(CONNECTION_TIMEOUT);
      }
   }

   public void unsubscribe()
   {
      requestMessage.setConnectionWanted(false);
      ros2.publish(streamRequestTopic, requestMessage);
      disconnect();
   }
}
