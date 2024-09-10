package us.ihmc.perception.streaming;

import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.SRTStreamMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.ros2.ROS2Input;

import java.net.InetSocketAddress;
import java.util.UUID;

import static us.ihmc.perception.streaming.StreamingTools.CONNECTION_TIMEOUT;
import static us.ihmc.perception.streaming.StreamingTools.STATUS_MESSAGE_UUID;

public class ROS2SRTVideoSubscriber
{
   private static final double UPDATE_TIMEOUT = 0.5; // half a second

   private final ROS2PublishSubscribeAPI ros2;
   private final ROS2IOTopicPair<SRTStreamMessage> streamMessageTopicPair;
   private final ROS2Input<SRTStreamMessage> statusMessageSubscriber;

   private final SRTStreamMessage requestMessage;

   private final SRTVideoSubscriber videoSubscriber;
   private final CameraIntrinsics cameraIntrinsics;
   private float depthDiscretization = 0.0f;

   private final Mat currentFrame = new Mat();
   private boolean receivedFirstFrame = false;
   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();

   public ROS2SRTVideoSubscriber(ROS2PublishSubscribeAPI ros2,
                                 ROS2IOTopicPair<SRTStreamMessage> streamMessageTopicPair,
                                 InetSocketAddress inputAddress,
                                 int outputAVPixelFormat)
   {

      this.ros2 = ros2;
      this.streamMessageTopicPair = streamMessageTopicPair;

      statusMessageSubscriber = ros2.subscribe(streamMessageTopicPair.getStatusTopic(),
                                               statusMessage -> STATUS_MESSAGE_UUID.equals(MessageTools.toUUID(statusMessage.getId())));

      requestMessage = new SRTStreamMessage();
      requestMessage.setReceiverAddress(inputAddress.getHostString());
      requestMessage.setReceiverPort(inputAddress.getPort());

      videoSubscriber = new SRTVideoSubscriber(inputAddress, outputAVPixelFormat);
      cameraIntrinsics = new CameraIntrinsics();
   }

   public boolean subscribe()
   {
      // Select UUID and create subscription to that ID
      UUID messageId = UUID.randomUUID();
      ROS2Input<SRTStreamMessage> ackMessageSubscription = ros2.subscribe(streamMessageTopicPair.getStatusTopic(),
                                                                          ackMessage -> messageId.equals(MessageTools.toUUID(ackMessage.getId())));

      // Send a request message
      MessageTools.toMessage(messageId, requestMessage.getId());
      requestMessage.setConnectionWanted(true);
      ros2.publish(streamMessageTopicPair.getCommandTopic(), requestMessage);

      // Wait to receive ACK
      // TODO: Add exception handler in case of interrupt
      SRTStreamMessage ackMessage = ackMessageSubscription.getMessageNotification().blockingPoll();

      // Subscribe via SRT
      boolean success = videoSubscriber.waitForConnection(CONNECTION_TIMEOUT);
      if (success)
      {
         cameraIntrinsics.setWidth(ackMessage.getImageWidth());
         cameraIntrinsics.setHeight(ackMessage.getImageHeight());
         cameraIntrinsics.setFx(ackMessage.getFx());
         cameraIntrinsics.setFy(ackMessage.getFy());
         cameraIntrinsics.setCx(ackMessage.getCx());
         cameraIntrinsics.setCy(ackMessage.getCy());
         depthDiscretization = ackMessage.getDepthDiscretization();
      }

      ackMessageSubscription.destroy();

      return success;
   }

   public void update()
   {
      if (statusMessageSubscriber.hasReceivedFirstMessage())
      {
         SRTStreamMessage statusMessage = statusMessageSubscriber.getLatest();
         sensorTransformToWorld.set(statusMessage.getOrientation(), statusMessage.getPosition());
      }

      Mat newFrame = videoSubscriber.getNextImage(UPDATE_TIMEOUT);
      if (newFrame != null)
      {
         newFrame.copyTo(currentFrame);
         receivedFirstFrame = true;
      }
   }

   public void unsubscribe()
   {
      if (!videoSubscriber.isConnected())
         return;

      UUID messageId = UUID.randomUUID();
      MessageTools.toMessage(messageId, requestMessage.getId());
      requestMessage.setConnectionWanted(false);
      ros2.publish(streamMessageTopicPair.getCommandTopic(), requestMessage);
      videoSubscriber.disconnect();
   }

   public void destroy()
   {
      unsubscribe();
      statusMessageSubscriber.destroy();
      videoSubscriber.destroy();
      currentFrame.close();
   }

   public boolean isConnected()
   {
      return videoSubscriber.isConnected();
   }

   public boolean hasCameraIntrinsics()
   {
      return statusMessageSubscriber.hasReceivedFirstMessage();
   }

   public boolean hasReceivedFirstFrame()
   {
      return receivedFirstFrame;
   }

   public Mat getCurrentFrame()
   {
      return currentFrame;
   }

   public RigidBodyTransformReadOnly getSensorTransformToWorld()
   {
      return sensorTransformToWorld;
   }

   public CameraIntrinsics getCameraIntrinsics()
   {
      return cameraIntrinsics;
   }

   public float getDepthDiscretization()
   {
      return depthDiscretization;
   }
}
