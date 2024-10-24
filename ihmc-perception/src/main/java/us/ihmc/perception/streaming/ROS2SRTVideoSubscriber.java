package us.ihmc.perception.streaming;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.SRTStreamStatus;
import perception_msgs.msg.dds.VideoFrameExtraData;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2Topic;

import java.time.Instant;
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
   private final PixelFormat nextFramePixelFormat;
   private final List<Consumer<RawImage>> newFrameConsumers = new ArrayList<>();
   private double lastFrameDelay;

   private final Thread subscriptionThread;
   private final AtomicBoolean subscriptionDesired = new AtomicBoolean(false);
   private volatile boolean shutdown = false;

   private final VideoFrameExtraData frameDataMessage = new VideoFrameExtraData();

   public ROS2SRTVideoSubscriber(ROS2PublishSubscribeAPI ros2, ROS2Topic<SRTStreamStatus> streamTopic, PixelFormat outputAVPixelFormat)
   {
      av_log_set_level(AV_LOG_FATAL); // silences no key frame errors which are 99% safe to ignore

      nextFramePixelFormat = outputAVPixelFormat;

      streamStatusMonitor = new ROS2StreamStatusMonitor(ros2, streamTopic);
      videoReceiver = new SRTVideoReceiver(outputAVPixelFormat.toFFmpegPixelFormat());
      subscriptionThread = ThreadTools.startAThread(this::subscriptionUpdate, "ROS2SRTVideoSubscription");
      
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy));
   }

   public void addNewFrameConsumer(Consumer<RawImage> newFrameConsumer)
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

   public double getLastFrameDelay()
   {
      return lastFrameDelay;
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
            if (nextFrame != null)
            {
               // Get frame data
               if (streamStatusMonitor.extraDataInStatusMessage())
                  frameDataMessage.set(streamStatusMonitor.getFrameExtraData());
               else
               {
                  BytePointer serializedFrameDataMessage = videoReceiver.getLastFrameSideData();
                  MessageTools.deserialize(serializedFrameDataMessage.asByteBuffer(), frameDataMessage);
               }

               FramePose3D frameSensorPose = new FramePose3D(frameDataMessage.getSensorPose());
               Instant frameAcquisitionTime = MessageTools.toInstant(frameDataMessage.getAcquisitionTime());
               lastFrameDelay = TimeTools.calculateDelay(frameAcquisitionTime);

               // Create a RawImage
               RawImage nextFrameRawImage = new RawImage(nextFrame,
                                                         null,
                                                         nextFramePixelFormat,
                                                         streamStatusMonitor.getCameraIntrinsics(),
                                                         frameSensorPose,
                                                         frameAcquisitionTime,
                                                         frameDataMessage.getSequenceNumber(),
                                                         streamStatusMonitor.getDepthDiscretization());

               // Hand it off to consumers
               newFrameConsumers.forEach(consumer -> consumer.accept(nextFrameRawImage));
               nextFrameRawImage.release();
            }
         }
         catch (InterruptedException ignored) {}
      }

      videoReceiver.destroy();
      if (nextFrame != null)
         nextFrame.close();
   }
}
