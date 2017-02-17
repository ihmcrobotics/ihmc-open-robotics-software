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

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.communication.packets.RequestLidarScanMessage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class LidarScanLogReader
{
   private static final boolean DEBUG = true;

   private static final NetClassList netClassList = new IHMCCommunicationKryoNetClassList();

   private DataInputStream logDataInputStream = null;

   private final String threadName = getClass().getSimpleName();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(threadName));
   private ScheduledFuture<?> currentLoggingTask = null;

   private PacketCommunicator packetCommunicator = null;
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
      if (packetCommunicator != null)
      {
         PrintTools.error(this, "There is already a log server running.");
      }
      else
      {
         packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(portToOpen, netClassList);
         packetCommunicator.connect();
         packetCommunicator.attachListener(RequestLidarScanMessage.class, message -> receivedLidarScanRequest.set(true));
      }

      if (currentLoggingTask == null)
         currentLoggingTask = executorService.scheduleAtFixedRate(this::readData, 0L, 10L, TimeUnit.MILLISECONDS);
   }

   public void stopServer()
   {
      if (packetCommunicator == null)
      {
         PrintTools.error(this, "The server has already been stopped.");
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
         PrintTools.error(this, "Already reading log.");
         return;
      }

      currentLogFileReference.set(logFile);

      if (packetCommunicator == null)
      {
         PrintTools.error(this, "No server running");
      }

      try
      {
         FileInputStream fileInputStream = new FileInputStream(logFile);
         logDataInputStream = new DataInputStream(fileInputStream);
         loggingEnabled.set(true);
         PrintTools.info(this, "Reading lidar log: " + logFile.getPath());
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

         if (packetCommunicator != null && lidarScanMessage != null)
            packetCommunicator.send(lidarScanMessage);
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
         lidarScanMessage.robotTimestamp = logDataInputStream.readLong();

         lidarScanMessage.lidarPosition = new Point3D32();
         lidarScanMessage.lidarPosition.setX(logDataInputStream.readFloat());
         lidarScanMessage.lidarPosition.setY(logDataInputStream.readFloat());
         lidarScanMessage.lidarPosition.setZ(logDataInputStream.readFloat());

         lidarScanMessage.lidarOrientation = new Quaternion32();
         double x = logDataInputStream.readFloat();
         double y = logDataInputStream.readFloat();
         double z = logDataInputStream.readFloat();
         double w = logDataInputStream.readFloat();
         lidarScanMessage.lidarOrientation.set(x, y, z, w);

         int scanLength = logDataInputStream.readInt();
         lidarScanMessage.scan = new float[scanLength];

         for (int i = 0; i < scanLength; i++)
         {
            lidarScanMessage.scan[i] = logDataInputStream.readFloat();
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
            PrintTools.info(this, "Finish loading.");
            loggingEnabled.set(false);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException("Could not close the input stream", e);
      }
   }
}
