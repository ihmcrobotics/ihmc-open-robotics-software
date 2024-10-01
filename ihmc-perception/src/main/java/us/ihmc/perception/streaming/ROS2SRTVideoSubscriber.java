package us.ihmc.perception.streaming;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.SRTStreamStatus;
import perception_msgs.msg.dds.StreamData;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import static org.bytedeco.ffmpeg.global.avutil.AV_LOG_FATAL;
import static org.bytedeco.ffmpeg.global.avutil.av_log_set_level;
import static us.ihmc.perception.streaming.StreamingTools.CONNECTION_TIMEOUT;

public class ROS2SRTVideoSubscriber
{
   private static final double UPDATE_TIMEOUT = 0.5; // half a second

   private final ROS2StreamStatusMonitor streamStatusMonitor;
   private final SRTVideoReceiver videoReceiver;

   private Mat nextFrame;
   private long frameTimestamp = -1;
   private final List<Consumer<Mat>> newFrameConsumers = new ArrayList<>();

   private final Thread subscriptionThread;
   private final AtomicBoolean subscriptionDesired = new AtomicBoolean(false);
   private volatile boolean shutdown = false;

   private final StreamData frameDataMessage = new StreamData();
   private final CameraIntrinsics cameraIntrinsics;
   private float depthDiscretization;
   private final MutableReferenceFrame sensorFrame;
   private final FramePose3D sensorPose; // updated upon access

   public ROS2SRTVideoSubscriber(ROS2PublishSubscribeAPI ros2, ROS2Topic<SRTStreamStatus> streamTopic, int outputAVPixelFormat)
   {
      av_log_set_level(AV_LOG_FATAL); // silences no key frame errors which are 99% safe to ignore

      cameraIntrinsics = new CameraIntrinsics();
      sensorFrame = new MutableReferenceFrame();
      sensorPose = new FramePose3D();
      
      streamStatusMonitor = new ROS2StreamStatusMonitor(ros2, streamTopic);
      videoReceiver = new SRTVideoReceiver(outputAVPixelFormat);
      subscriptionThread = ThreadTools.startAThread(this::subscriptionUpdate, "ROS2SRTVideoSubscription");
      
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy));
   }

   public void addNewFrameConsumer(Consumer<Mat> newFrameConsumer)
   {
      newFrameConsumers.add(newFrameConsumer);
   }

   public void subscribe()
   {
      synchronized (subscriptionDesired)
      {
         subscriptionDesired.set(true);
         subscriptionDesired.notify();
      }
   }

   public void unsubscribe()
   {
      synchronized (subscriptionDesired)
      {
         subscriptionDesired.set(false);
         subscriptionThread.interrupt();
      }
   }

   public void destroy()
   {
      shutdown = true;
      unsubscribe();
      try
      {
         subscriptionThread.join();
      }
      catch (InterruptedException exception)
      {
         LogTools.error("Interrupted while waiting for {} thread to shut down.", subscriptionThread.getName());
      }
   }

   public boolean isConnected()
   {
      return videoReceiver.isConnected();
   }

   public ReferenceFrame getSensorFrame()
   {
      return sensorFrame.getReferenceFrame();
   }

   public RigidBodyTransformReadOnly getSensorTransformToWorld()
   {
      return sensorFrame.getTransformToParent();
   }

   public FramePose3DReadOnly getSensorPose()
   {
      synchronized (sensorFrame)
      {
         sensorPose.setToZero(getSensorFrame());
      }
      sensorPose.changeFrame(ReferenceFrame.getWorldFrame());
      return sensorPose;
   }

   public CameraIntrinsics getCameraIntrinsics()
   {
      return cameraIntrinsics;
   }

   public float getDepthDiscretization()
   {
      return depthDiscretization;
   }

   public long getFrameTimestamp()
   {
      return frameTimestamp;
   }

   private void subscriptionUpdate()
   {
      while (!shutdown)
      {
         try
         {
            // If we don't want to be subscribed, wait until subscription is desired
            if (!subscriptionDesired.get())
            {
               videoReceiver.disconnect();
               synchronized (subscriptionDesired)
               {
                  subscriptionDesired.wait();
               }
               continue;
            }

            // SUBSCRIPTION IS DESIRED
            // If there's no stream, wait for the stream
            if (!streamStatusMonitor.isStreaming())
            {
               videoReceiver.disconnect();
               streamStatusMonitor.waitForStream(UPDATE_TIMEOUT);
               continue;
            }

            // VIDEO IS BEING STREAMED
            // If we're not connected, try to connect
            if (!videoReceiver.isConnected())
            {
               videoReceiver.connect(streamStatusMonitor.getStreamerAddress(), CONNECTION_TIMEOUT);
               continue;
            }

            // RECEIVER IS CONNECTED
            // Get the latest image and give it to consumers
            nextFrame = videoReceiver.getNextFrame(UPDATE_TIMEOUT);
            frameTimestamp = videoReceiver.getLastFrameTimestamp();
            if (nextFrame != null)
            {
               BytePointer serializedFrameDataMessage = videoReceiver.getLastFrameSideData();
               MessageTools.deserialize(serializedFrameDataMessage.asByteBuffer(), frameDataMessage);

               cameraIntrinsics.setWidth(frameDataMessage.getImageWidth());
               cameraIntrinsics.setHeight(frameDataMessage.getImageHeight());
               cameraIntrinsics.setFx(frameDataMessage.getFx());
               cameraIntrinsics.setFy(frameDataMessage.getFy());
               cameraIntrinsics.setCx(frameDataMessage.getCx());
               cameraIntrinsics.setCy(frameDataMessage.getCy());
               depthDiscretization = frameDataMessage.getDepthDiscretization();

               synchronized (sensorFrame)
               {
                  sensorFrame.update(transformToWorld -> transformToWorld.set(frameDataMessage.getOrientation(), frameDataMessage.getPosition()));
               }

               newFrameConsumers.forEach(consumer -> consumer.accept(nextFrame));
               nextFrame.close();
            }
         }
         catch (InterruptedException ignored) {}
      }

      videoReceiver.destroy();
      if (nextFrame != null)
         nextFrame.close();
   }
}
