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

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.communication.packets.RequestLidarScanMessage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class LidarScanLogWriter
{
   private static final boolean DEBUG = false;

   private static final NetworkPorts port = NetworkPorts.LIDAR_SCAN_LOGGER_PORT;
   private static final NetClassList netClassList = new IHMCCommunicationKryoNetClassList();

   private DataOutputStream logDataOutputStream = null;

   private final String threadName = getClass().getSimpleName();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(threadName));
   private ScheduledFuture<?> currentLoggingTask = null;

   private PacketCommunicator packetCommunicator = null;

   private final AtomicReference<LidarScanMessage> newMessage = new AtomicReference<>(null);
   private PacketConsumer<LidarScanMessage> lidarScanConsumer = null;

   private final AtomicBoolean loggingEnabled = new AtomicBoolean(false);

   public LidarScanLogWriter()
   {
   }

   public void connectToNetworkProcessor(String host) throws IOException
   {
      if (packetCommunicator != null)
      {
         PrintTools.error(this, "The logger is already connected to the network processor.");
      }
      else
      {
         packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(host, port, netClassList);
         packetCommunicator.attachListener(LidarScanMessage.class, this::receiveLidarScanMessage);
         packetCommunicator.connect();
      }

      if (currentLoggingTask == null)
         currentLoggingTask = executorService.scheduleAtFixedRate(this::writeData, 0L, 1L, TimeUnit.MILLISECONDS);
   }

   public void disconnectFromNetworkProcessor()
   {
      if (packetCommunicator == null)
      {
         PrintTools.error(this, "The logger is already disconnected from the network processor.");
      }
      else
      {
         packetCommunicator.closeConnection();
         packetCommunicator.close();
         packetCommunicator = null;
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
         PrintTools.error(this, "Already writing data.");
         return;
      }

      try
      {
         FileOutputStream fileOutputStream = new FileOutputStream(logFile);
         logDataOutputStream = new DataOutputStream(fileOutputStream);
         loggingEnabled.set(true);
         PrintTools.info(this, "Recording lidar log: " + logFile.getPath());
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
      if (!packetCommunicator.isConnected())
         return;

      packetCommunicator.send(new RequestLidarScanMessage());

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
            PrintTools.info(this, "Finish recording.");
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
         logDataOutputStream.writeLong(lidarScanMessage.robotTimestamp);

         logDataOutputStream.writeFloat(lidarScanMessage.lidarPosition.x);
         logDataOutputStream.writeFloat(lidarScanMessage.lidarPosition.y);
         logDataOutputStream.writeFloat(lidarScanMessage.lidarPosition.z);

         logDataOutputStream.writeFloat(lidarScanMessage.lidarOrientation.x);
         logDataOutputStream.writeFloat(lidarScanMessage.lidarOrientation.y);
         logDataOutputStream.writeFloat(lidarScanMessage.lidarOrientation.z);
         logDataOutputStream.writeFloat(lidarScanMessage.lidarOrientation.w);

         logDataOutputStream.writeInt(lidarScanMessage.scan.length);

         for (float scanPoint : lidarScanMessage.scan)
         {
            logDataOutputStream.writeFloat(scanPoint);
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
