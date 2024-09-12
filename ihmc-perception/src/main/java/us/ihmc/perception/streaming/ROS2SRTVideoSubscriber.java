package us.ihmc.perception.streaming;

import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.SRTStreamMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.tools.thread.MissingThreadTools;

import java.net.InetSocketAddress;
import java.util.UUID;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

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

   private final SRTStreamMessage requestMessage;

   private final ExecutorService taskExecutor = Executors.newFixedThreadPool(4);
   private Future<?> subscriptionMaintenanceTask;
   private Future<?> connectionRequestTask;
   private Future<?> subscriptionTask;
   private Future<?> unsubscriptionTask;

   private volatile boolean connectionDesired = false;
   private final Lock connectionLock = new ReentrantLock();
   private final Condition connectionDesiredCondition = connectionLock.newCondition();

   private final SRTVideoReceiver videoReceiver;
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

      statusMessageSubscription = ros2.subscribe(streamMessageTopicPair.getStatusTopic(),
                                                 statusMessage -> STATUS_MESSAGE_UUID.equals(MessageTools.toUUID(statusMessage.getId())));
      ackMessageSubscription = ros2.subscribe(streamMessageTopicPair.getStatusTopic(),
                                              ackMessage -> subscriberId.equals(MessageTools.toUUID(ackMessage.getId())));
      ackMessageSubscription.addCallback(this::receiveACKMessage);

      requestMessage = new SRTStreamMessage();
      requestMessage.setReceiverAddress(inputAddress.getHostString());
      requestMessage.setReceiverPort(inputAddress.getPort());

      videoReceiver = new SRTVideoReceiver(inputAddress, outputAVPixelFormat);
      cameraIntrinsics = new CameraIntrinsics();

      subscriptionMaintenanceTask = taskExecutor.submit(this::maintainConnection);
   }

   public void subscribe()
   {
      cancelSubscriptionTasks();
      subscriptionTask = taskExecutor.submit(() ->
      {
         connectionLock.lock();
         try
         {
            LogTools.warn("Signaling connection desired");
            connectionDesired = true;
            connectionDesiredCondition.signal();
         }
         finally
         {
            connectionLock.unlock();
         }
      });
   }

   private void maintainConnection()
   {
      while (true)
      {
         connectionLock.lock();
         try
         {
            while (!connectionDesired)
               connectionDesiredCondition.await();

            if (!videoReceiver.isConnected())
            {
               LogTools.warn("Sending connection request");
               connectionRequestTask = taskExecutor.submit(this::sendConnectionRequest);
               connectionRequestTask.get();
            }
         }
         catch (InterruptedException e)
         {
            LogTools.error("Maintenance Interrupted");
            break;
         }
         catch (ExecutionException e)
         {
            LogTools.error("Request Execution Failed");
            break;
         }
         finally
         {
            connectionLock.unlock();
         }

         MissingThreadTools.sleep(UPDATE_TIMEOUT);
      }
   }

   private void sendConnectionRequest()
   {
      // Send a request message
      MessageTools.toMessage(subscriberId, requestMessage.getId());
      requestMessage.setConnectionWanted(true);
      ros2.publish(streamMessageTopicPair.getCommandTopic(), requestMessage);

      // Wait to receive ACK
      synchronized (ackMessageReceivedNotification)
      {
         try
         {
            LogTools.warn("Waiting to receive ACK");
            ackMessageReceivedNotification.wait((long) Conversions.secondsToMilliseconds(CONNECTION_TIMEOUT));
            LogTools.warn("Stopped waiting for ACK");
         }
         catch (InterruptedException ignored)
         {
            LogTools.error("Send Connection Interrupted");
            return;
         }
      }

      // Subscribe via SRT
      videoReceiver.waitForConnection(CONNECTION_TIMEOUT);
   }

   private void receiveACKMessage(SRTStreamMessage ackMessage)
   {
      synchronized (ackMessageReceivedNotification)
      {
         LogTools.warn("Received ACK");
         ackMessageReceivedNotification.notify();
      }

      // get camera intrinsics from ACK message
      cameraIntrinsics.setWidth(ackMessage.getImageWidth());
      cameraIntrinsics.setHeight(ackMessage.getImageHeight());
      cameraIntrinsics.setFx(ackMessage.getFx());
      cameraIntrinsics.setFy(ackMessage.getFy());
      cameraIntrinsics.setCx(ackMessage.getCx());
      cameraIntrinsics.setCy(ackMessage.getCy());
      depthDiscretization = ackMessage.getDepthDiscretization();
   }

   public void update()
   {
      connectionLock.lock();
      try
      {
         if (!videoReceiver.isConnected() || !connectionDesired)
            return;

         LogTools.warn("Updating");
         Mat newFrame = videoReceiver.getNextImage(UPDATE_TIMEOUT);
         if (newFrame != null)
         {
            newFrame.copyTo(currentFrame);
            receivedFirstFrame = true;
         }
      }
      finally
      {
         connectionLock.unlock();
      }

      if (statusMessageSubscription.hasReceivedFirstMessage())
      {
         SRTStreamMessage statusMessage = statusMessageSubscription.getLatest();
         sensorTransformToWorld.set(statusMessage.getOrientation(), statusMessage.getPosition());
      }
   }

   public void unsubscribe()
   {
      cancelSubscriptionTasks();
      unsubscriptionTask = taskExecutor.submit(() ->
      {
         try
         {
            connectionLock.lockInterruptibly();

            LogTools.warn("Unsubscribing");
            connectionDesired = false;

            UUID messageId = UUID.randomUUID();
            MessageTools.toMessage(messageId, requestMessage.getId());
            requestMessage.setConnectionWanted(false);
            ros2.publish(streamMessageTopicPair.getCommandTopic(), requestMessage);
            videoReceiver.disconnect();
         }
         catch (InterruptedException ignored) {}
         finally
         {
            connectionLock.unlock();
         }
      });
   }

   private void cancelSubscriptionTasks()
   {
      if (connectionRequestTask != null)
         connectionRequestTask.cancel(true);
      if (subscriptionTask != null)
         subscriptionTask.cancel(true);
      if (unsubscriptionTask != null)
         unsubscriptionTask.cancel(true);
   }

   public void destroy()
   {
      unsubscribe();

      taskExecutor.shutdown();

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
