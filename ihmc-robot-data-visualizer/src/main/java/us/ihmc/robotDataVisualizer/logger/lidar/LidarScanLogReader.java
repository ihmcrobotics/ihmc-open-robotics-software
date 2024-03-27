package us.ihmc.robotDataVisualizer.logger.lidar;

import java.io.DataInputStream;
import java.io.EOFException;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeROS2Node;

public class LidarScanLogReader
{
   private static final boolean DEBUG = true;

   private DataInputStream logDataInputStream = null;

   private final String threadName = getClass().getSimpleName();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(threadName));
   private ScheduledFuture<?> currentLoggingTask = null;

   private RealtimeROS2Node ros2Node;
   private ROS2PublisherBasics<LidarScanMessage> lidarScanPublisher;
   private PacketConsumer<LidarScanMessage> lidarScanConsumer = null;

   private final AtomicBoolean loggingEnabled = new AtomicBoolean(false);
   private final AtomicBoolean pauseReading = new AtomicBoolean(false);
   private final AtomicBoolean reloadLog = new AtomicBoolean(false);
   private final AtomicBoolean waitForListener = new AtomicBoolean(false);
   private final AtomicReference<File> currentLogFileReference = new AtomicReference<>(null);

   private final AtomicBoolean receivedLidarScanRequest = new AtomicBoolean(true);

   public LidarScanLogReader()
   {
   }

   public void startServer(NetworkPorts portToOpen) throws IOException
   {
      if (ros2Node != null)
      {
         LogTools.error("There is already a log server running.");
      }
      else
      {
         ros2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.FAST_RTPS, "lidar_log");
         lidarScanPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, LidarScanMessage.class, ROS2Tools.IHMC_ROOT);
         ros2Node.spin();
      }

      if (currentLoggingTask == null)
         currentLoggingTask = executorService.scheduleAtFixedRate(this::readData, 0L, 10L, TimeUnit.MILLISECONDS);
   }

   public void stopServer()
   {
      if (ros2Node == null)
      {
         LogTools.error("The server has already been stopped.");
      }
      else
      {
         lidarScanPublisher = null;
         ros2Node.destroy();
         ros2Node = null;
      }

      if (currentLoggingTask != null)
      {
         currentLoggingTask.cancel(false);
         currentLoggingTask = null;
      }
   }

   public void stopExecutor()
   {
      executorService.shutdown();

      try
      {
         executorService.awaitTermination(10, TimeUnit.SECONDS);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException("Cannot shutdown " + threadName, e);
      }
   }

   public void pauseReading(boolean pause)
   {
      pauseReading.set(pause);
   }

   public void reloadLogFile()
   {
      reloadLog.set(true);
   }

   public void waitForListener(boolean value)
   {
      waitForListener.set(value);
   }

   public void setLidarScanMessageConsumer(PacketConsumer<LidarScanMessage> consumer)
   {
      lidarScanConsumer = consumer;
   }

   public String getCurrentLogFileFullName()
   {
      if (currentLogFileReference.get() == null)
         return "None.";
      else
         return currentLogFileReference.get().getPath();
   }

   public void startReading(File logFile)
   {
      if (loggingEnabled.get())
      {
         LogTools.error("Already reading log.");
         return;
      }

      currentLogFileReference.set(logFile);

      if (ros2Node == null)
      {
         LogTools.error("No server running");
      }

      try
      {
         FileInputStream fileInputStream = new FileInputStream(logFile);
         logDataInputStream = new DataInputStream(fileInputStream);
         loggingEnabled.set(true);
         LogTools.info("Reading lidar log: " + logFile.getPath());
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
         logDataInputStream = null;
         loggingEnabled.set(false);
      }
   }

   public void stopReading()
   {
      loggingEnabled.set(false);
   }

   private void readData()
   {
      try
      {
         if (reloadLog.get())
         {
            closeDataInputStream();
            startReading(currentLogFileReference.get());
            reloadLog.set(false);
            return;
         }

         if (pauseReading.get() || logDataInputStream == null)
            return;

         if (!loggingEnabled.get())
         {
            closeDataInputStream();
            return;
         }

         if (waitForListener.get() && !receivedLidarScanRequest.getAndSet(false))
            return;

         LidarScanMessage lidarScanMessage = readMessage();
         lidarScanConsumer.receivedPacket(lidarScanMessage);

         if (lidarScanPublisher != null && lidarScanMessage != null)
            lidarScanPublisher.publish(lidarScanMessage);
      }
      catch (Exception e)
      {
         if (DEBUG)
            e.printStackTrace();
      }
   }

   private LidarScanMessage readMessage()
   {
      if (!loggingEnabled.get() || logDataInputStream == null)
         return null;

      try
      {
         LidarScanMessage lidarScanMessage = new LidarScanMessage();
         lidarScanMessage.setRobotTimestamp(logDataInputStream.readLong());

         lidarScanMessage.getLidarPosition().setX(logDataInputStream.readFloat());
         lidarScanMessage.getLidarPosition().setY(logDataInputStream.readFloat());
         lidarScanMessage.getLidarPosition().setZ(logDataInputStream.readFloat());

         double x = logDataInputStream.readFloat();
         double y = logDataInputStream.readFloat();
         double z = logDataInputStream.readFloat();
         double w = logDataInputStream.readFloat();
         lidarScanMessage.getLidarOrientation().set(x, y, z, w);

         int scanDataLength = logDataInputStream.readInt();

         for (int i = 0; i < scanDataLength; i++)
         {
            lidarScanMessage.getScan().add(logDataInputStream.readByte());
         }
         return lidarScanMessage;
      }
      catch (EOFException e)
      {
         closeDataInputStream();
         return null;
      }
      catch (IOException e)
      {
         if (DEBUG)
            e.printStackTrace();
         closeDataInputStream();
         return null;
      }
   }

   private void closeDataInputStream()
   {
      try
      {
         if (logDataInputStream != null)
         {
            logDataInputStream.close();
            logDataInputStream = null;
            LogTools.info("Finish loading.");
            loggingEnabled.set(false);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException("Could not close the input stream", e);
      }
   }
}
