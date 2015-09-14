package us.ihmc.robotDataCommunication;

import us.ihmc.multicastLogDataProtocol.LogPacketHandler;
import us.ihmc.multicastLogDataProtocol.StreamingDataTCPClient;
import us.ihmc.multicastLogDataProtocol.ThreadedLogPacketHandler;
import us.ihmc.multicastLogDataProtocol.control.LogHandshake;
import us.ihmc.tools.compression.SnappyUtils;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.zip.CRC32;

public abstract class YoVariableDataReceiver implements LogPacketHandler
{
   private static final int RECEIVE_BUFFER_SIZE = 1024;

   private ByteBuffer decompressed;
   private long previous;
   private int displayOneInNPackets;

   private CRC32 crc32 = new CRC32();
   private final StreamingDataTCPClient client;
   private final ThreadedLogPacketHandler updateHandler;

   public YoVariableDataReceiver(byte[] dataIP, int port, int displayOneInNPackets)
   {
      InetAddress inetAddress;
      this.displayOneInNPackets = displayOneInNPackets;

      try
      {
         inetAddress = InetAddress.getByAddress(dataIP);
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }

      updateHandler = new ThreadedLogPacketHandler(this, RECEIVE_BUFFER_SIZE);
      client = new StreamingDataTCPClient(inetAddress, port, updateHandler, displayOneInNPackets);
   }

   public void start(int bufferSize)
   {
      decompressed = ByteBuffer.allocate(bufferSize);

      updateHandler.start();
      client.start();
   }

   public void stop()
   {
      if (client.isRunning())
      {
         client.requestStop();
      }
   }

   public LogHandshake getHandshake()
   {
      try
      {
         return client.getHandshake();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   @Override
   public void newDataAvailable(LogDataHeader header, ByteBuffer buf)
   {
      if (header.getUid() > previous + displayOneInNPackets)
      {
         System.err.println("Skipped " + (header.getUid() - previous - displayOneInNPackets) + " packets");
      }
      else if (header.getUid() <= previous)
      {
         System.err.println("Packet skew detected " + header.getUid());
      }

      previous = header.getUid();
      decompressed.clear();
      buf.clear();

      long checksum = header.getCrc32() & 0xFFFFFFFFL;
      crc32.reset();
      crc32.update(buf.array(), buf.position() + buf.arrayOffset(), buf.remaining());

      if (crc32.getValue() != checksum)
      {
         System.err.println("[" + getClass().getSimpleName() + "] Checksum validation failure. Ignoring packet " + header.getUid() + ".");
         return;
      }

      try
      {
         SnappyUtils.uncompress(buf, decompressed);
         decompressed.flip();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return;
      }

      onNewData(decompressed);
   }

   @Override
   public void timestampReceived(long timestamp)
   {
      onNewTimestamp(timestamp);
   }

   @Override
   public void timeout()
   {
      updateHandler.shutdown();
      onTimeout();
   }

   public abstract void onNewData(ByteBuffer data);

   public abstract void onNewTimestamp(long timestamp);

   public abstract void onTimeout();
}