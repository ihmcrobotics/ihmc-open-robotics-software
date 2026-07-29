package us.ihmc.perception.streaming;

import perception_msgs.SRTStreamStatus;
import perception_msgs.VideoFrameExtraData;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.perception.CameraModel;
import us.ihmc.sensors.CameraIntrinsics;
import us.ihmc.tools.Timer;

import java.net.InetSocketAddress;
import java.util.concurrent.atomic.AtomicBoolean;

public class ROS2StreamStatusMonitor
{
   public static final double MESSAGE_EXPIRATION_MULTIPLIER = 5;

   private final Thread messageMonitor;
   private final Throttler throttler;
   private final Timer messageTimer;

   private InetSocketAddress streamerAddress;
   private double expectedUpdatePeriod = Conversions.hertzToSeconds(30.0);
   private final AtomicBoolean isStreaming;

   private final CameraIntrinsics cameraIntrinsics;
   private CameraModel cameraModel;
   private float depthDiscretization;
   private VideoFrameExtraData frameExtraData;

   private boolean running = true;

   public ROS2StreamStatusMonitor(ROS2Node ros2Node, ROS2Topic<SRTStreamStatus> streamTopic)
   {
      isStreaming = new AtomicBoolean(false);
      cameraIntrinsics = new CameraIntrinsics();

      throttler = new Throttler();
      messageTimer = new Timer();
      messageMonitor = ThreadTools.startAsDaemon(this::monitorMessageFrequency, "StreamStatusMonitor-" + streamTopic.getName());

      ros2Node.createSubscriptionSampler(streamTopic, this::receiveMessage);
   }

   public InetSocketAddress getStreamerAddress()
   {
      return streamerAddress;
   }

   public boolean isStreaming()
   {
      return isStreaming.get();
   }

   public void waitForStream()
   {
      waitForStream(DefaultExceptionHandler.PRINT_MESSAGE);
   }

   public void waitForStream(ExceptionHandler exceptionHandler)
   {
      if (isStreaming.get())
         return;

      synchronized (isStreaming)
      {
         ExceptionTools.handle(() -> isStreaming.wait(), exceptionHandler);
      }
   }

   public void waitForStream(double timeout)
   {
      waitForStream(timeout, DefaultExceptionHandler.PROCEED_SILENTLY);
   }

   public void waitForStream(double timeout, ExceptionHandler exceptionHandler)
   {
      if (isStreaming.get())
         return;

      synchronized (isStreaming)
      {
         ExceptionTools.handle(() -> isStreaming.wait((long) Conversions.secondsToMilliseconds(timeout)), exceptionHandler);
      }
   }

   public CameraIntrinsics getCameraIntrinsics()
   {
      return cameraIntrinsics;
   }

   public CameraModel getCameraModel()
   {
      return cameraModel;
   }

   public boolean extraDataInStatusMessage()
   {
      return frameExtraData != null;
   }

   public VideoFrameExtraData getFrameExtraData()
   {
      return frameExtraData;
   }

   public float getDepthDiscretization()
   {
      return depthDiscretization;
   }

   public void destroy()
   {
      running = false;
      messageMonitor.interrupt();
   }

   private void receiveMessage(SRTStreamStatus statusMessage)
   {
      messageTimer.reset();

      streamerAddress = InetSocketAddress.createUnresolved(statusMessage.getStreamerAddressAsString(), statusMessage.getStreamerPort());
      expectedUpdatePeriod = Conversions.hertzToSeconds(statusMessage.getExpectedPublishFrequency());

      cameraIntrinsics.setWidth(statusMessage.getImageWidth());
      cameraIntrinsics.setHeight(statusMessage.getImageHeight());
      cameraIntrinsics.setFx(statusMessage.getFx());
      cameraIntrinsics.setFy(statusMessage.getFy());
      cameraIntrinsics.setCx(statusMessage.getCx());
      cameraIntrinsics.setCy(statusMessage.getCy());
      cameraModel = CameraModel.fromByte(statusMessage.getCameraModel());
      depthDiscretization = statusMessage.getDepthDiscretization();

      if (statusMessage.getContainsExtraData())
         frameExtraData = statusMessage.getFrameExtraData();

      synchronized (isStreaming)
      {
         boolean wasStreaming = isStreaming.getAndSet(statusMessage.getIsStreaming());
         if (isStreaming.get() && !wasStreaming)
            isStreaming.notifyAll();
      }
   }

   private void monitorMessageFrequency()
   {
      while (running)
      {
         waitForStream();

         if (isStreaming.get())
         {
            throttler.waitAndRun(expectedUpdatePeriod);
            boolean messageValid = messageTimer.isRunning(MESSAGE_EXPIRATION_MULTIPLIER * expectedUpdatePeriod);
            isStreaming.compareAndSet(true, messageValid);
         }
      }
   }
}
