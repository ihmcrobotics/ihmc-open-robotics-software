package us.ihmc.robotDataVisualizer.logger.lidar;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Subscription;

public class LidarScanLogWriter
{
   private static final boolean DEBUG = false;

   private DataOutputStream logDataOutputStream = null;

   private final String threadName = getClass().getSimpleName();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(threadName));
   private ScheduledFuture<?> currentLoggingTask = null;

   private final AtomicReference<LidarScanMessage> newMessage = new AtomicReference<>(null);
   private PacketConsumer<LidarScanMessage> lidarScanConsumer = null;

   private final AtomicBoolean loggingEnabled = new AtomicBoolean(false);

   private final ROS2Node ros2Node;
   private ROS2Subscription<LidarScanMessage> subscription;

   public LidarScanLogWriter(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   public void connectToNetworkProcessor(String topicName) throws IOException
   {
      if (subscription != null)
      {
         LogTools.error("The logger is already connected to the network processor.");
      }
      else
      {
         subscription = ros2Node.createSubscription(LidarScanMessage.class, subscriber -> receiveLidarScanMessage(subscriber.takeNextData()), topicName);
      }

      if (currentLoggingTask == null)
         currentLoggingTask = executorService.scheduleAtFixedRate(this::writeData, 0L, 1L, TimeUnit.MILLISECONDS);
   }

   public void disconnectFromNetworkProcessor()
   {
      if (subscription == null)
      {
         LogTools.error("The logger is already disconnected from the network processor.");
      }
      else
      {
         subscription.remove();
         subscription = null;
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

   public void setLidarScanMessageConsumer(PacketConsumer<LidarScanMessage> consumer)
   {
      lidarScanConsumer = consumer;
   }

   public void startWriting(File logFile)
   {
      if (loggingEnabled.get())
      {
         LogTools.error("Already writing data.");
         return;
      }

      try
      {
         FileOutputStream fileOutputStream = new FileOutputStream(logFile);
         logDataOutputStream = new DataOutputStream(fileOutputStream);
         loggingEnabled.set(true);
         LogTools.info("Recording lidar log: " + logFile.getPath());
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
         logDataOutputStream = null;
         loggingEnabled.set(false);
      }
   }

   public void stopWriting()
   {
      loggingEnabled.set(false);
   }

   private void writeData()
   {
      if (subscription == null)
         return;

      LidarScanMessage lidarScanMessage = newMessage.getAndSet(null);

      if (!loggingEnabled.get())
         closeDataOutputStream();

      if (lidarScanMessage == null)
         return;

      writeMessage(lidarScanMessage);
      lidarScanConsumer.receivedPacket(lidarScanMessage);
   }

   private void closeDataOutputStream()
   {
      try
      {
         if (logDataOutputStream != null)
         {
            logDataOutputStream.close();
            logDataOutputStream = null;
            LogTools.info("Finish recording.");
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException("Could not close the output stream", e);
      }
   }

   private void writeMessage(LidarScanMessage lidarScanMessage)
   {
      if (!loggingEnabled.get() || logDataOutputStream == null)
         return;

      try
      {
         logDataOutputStream.writeLong(lidarScanMessage.getRobotTimestamp());

         logDataOutputStream.writeFloat(lidarScanMessage.getLidarPosition().getX32());
         logDataOutputStream.writeFloat(lidarScanMessage.getLidarPosition().getY32());
         logDataOutputStream.writeFloat(lidarScanMessage.getLidarPosition().getZ32());

         logDataOutputStream.writeFloat(lidarScanMessage.getLidarOrientation().getX32());
         logDataOutputStream.writeFloat(lidarScanMessage.getLidarOrientation().getY32());
         logDataOutputStream.writeFloat(lidarScanMessage.getLidarOrientation().getZ32());
         logDataOutputStream.writeFloat(lidarScanMessage.getLidarOrientation().getS32());

         logDataOutputStream.writeInt(lidarScanMessage.getScan().size());

         for (byte scanData : lidarScanMessage.getScan().toArray())
         {
            logDataOutputStream.writeByte(scanData);
         }
      }
      catch (IOException e)
      {
         if (DEBUG)
            e.printStackTrace();
         closeDataOutputStream();
         loggingEnabled.set(false);
      }
   }

   private void receiveLidarScanMessage(LidarScanMessage message)
   {
      newMessage.set(message);
   }
}
