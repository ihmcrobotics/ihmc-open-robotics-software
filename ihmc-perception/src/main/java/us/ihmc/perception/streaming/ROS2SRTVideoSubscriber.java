package us.ihmc.perception.streaming;

import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.SRTStreamMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
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

   private final UUID subscriberId = UUID.randomUUID();

   private final ROS2PublishSubscribeAPI ros2;
   private final ROS2IOTopicPair<SRTStreamMessage> streamMessageTopicPair;
   private final ROS2Input<SRTStreamMessage> statusMessageSubscription;
   private final ROS2Input<SRTStreamMessage> ackMessageSubscription;
   private final Object ackMessageReceivedNotification = new Object();
   private Thread subscriptionMaintainer;

   private final SRTStreamMessage requestMessage;
   private final SRTVideoReceiver videoReceiver;

   private final CameraIntrinsics cameraIntrinsics;
   private float depthDiscretization = 0.0f;

   private final Mat currentFrame = new Mat();
   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();

   private final Notification newFrameReceived = new Notification();
   private volatile boolean connectionDesired = false;

   public ROS2SRTVideoSubscriber(ROS2PublishSubscribeAPI ros2,
                                 ROS2IOTopicPair<SRTStreamMessage> streamMessageTopicPair,
                                 InetSocketAddress inputAddress,
                                 int outputAVPixelFormat)
   {
      this.ros2 = ros2;
      this.streamMessageTopicPair = streamMessageTopicPair;

      statusMessageSubscription = ros2.subscribe(streamMessageTopicPair.getStatusTopic(),
                                                 statusMessage -> STATUS_MESSAGE_UUID.equals(MessageTools.toUUID(statusMessage.getId())));
      ackMessageSubscription = ros2.subscribe(streamMessageTopicPair.getStatusTopic(),
                                              ackMessage -> subscriberId.equals(MessageTools.toUUID(ackMessage.getId())));
      ackMessageSubscription.addCallback(this::receiveACKMessage);

      requestMessage = new SRTStreamMessage();
      requestMessage.setReceiverAddress(inputAddress.getHostString());
      requestMessage.setReceiverPort(inputAddress.getPort());
      MessageTools.toMessage(subscriberId, requestMessage.getId());

      videoReceiver = new SRTVideoReceiver(inputAddress, outputAVPixelFormat);
      cameraIntrinsics = new CameraIntrinsics();
   }

   public synchronized void subscribe()
   {
      connectionDesired = true;

      stopMaintenanceThread();
      subscriptionMaintainer = ThreadTools.startAsDaemon(this::maintainConnection, "ROS2SRTVideoSubscriptionMaintainer");
   }

   public synchronized void update()
   {
      if (!videoReceiver.isConnected() || !connectionDesired)
         return;

      Mat newFrame = videoReceiver.getNextImage(UPDATE_TIMEOUT);
      if (newFrame != null)
      {
         newFrame.copyTo(currentFrame);
         newFrameReceived.set();
      }

      if (statusMessageSubscription.hasReceivedFirstMessage())
      {
         SRTStreamMessage statusMessage = statusMessageSubscription.getLatest();
         sensorTransformToWorld.set(statusMessage.getOrientation(), statusMessage.getPosition());
      }
   }

   public synchronized void unsubscribe()
   {
      connectionDesired = false;
      stopMaintenanceThread();

      requestMessage.setConnectionWanted(false);
      ros2.publish(streamMessageTopicPair.getCommandTopic(), requestMessage);
      videoReceiver.disconnect();
   }

   public void destroy()
   {
      unsubscribe();

      statusMessageSubscription.destroy();
      ackMessageSubscription.destroy();
      videoReceiver.destroy();
      currentFrame.close();
   }

   public boolean isConnected()
   {
      return videoReceiver.isConnected();
   }

   public boolean hasCameraIntrinsics()
   {
      return statusMessageSubscription.hasReceivedFirstMessage();
   }

   public boolean newFrameAvailable()
   {
      return newFrameReceived.poll();
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

   private void maintainConnection()
   {
      try
      {
         while (connectionDesired)
         {
            if (!videoReceiver.isConnected())
            {
               sendConnectionRequest();

               synchronized (ackMessageReceivedNotification)
               {
                  ackMessageReceivedNotification.wait((long) Conversions.secondsToMilliseconds(CONNECTION_TIMEOUT));
               }
            }

            Thread.sleep((long) Conversions.secondsToMilliseconds(UPDATE_TIMEOUT));
         }
      }
      catch (InterruptedException ignored) {}
   }

   private void sendConnectionRequest()
   {
      // Send a request message
      requestMessage.setConnectionWanted(true);
      ros2.publish(streamMessageTopicPair.getCommandTopic(), requestMessage);
   }

   private void receiveACKMessage(SRTStreamMessage ackMessage)
   {
      if (connectionDesired && !videoReceiver.isConnected())
      {
         videoReceiver.waitForConnection(CONNECTION_TIMEOUT);

         // get camera intrinsics from ACK message
         cameraIntrinsics.setWidth(ackMessage.getImageWidth());
         cameraIntrinsics.setHeight(ackMessage.getImageHeight());
         cameraIntrinsics.setFx(ackMessage.getFx());
         cameraIntrinsics.setFy(ackMessage.getFy());
         cameraIntrinsics.setCx(ackMessage.getCx());
         cameraIntrinsics.setCy(ackMessage.getCy());
         depthDiscretization = ackMessage.getDepthDiscretization();
      }

      synchronized (ackMessageReceivedNotification)
      {
         ackMessageReceivedNotification.notify();
      }
   }

   private void stopMaintenanceThread()
   {
      if (subscriptionMaintainer != null)
         subscriptionMaintainer.interrupt();
   }
}
