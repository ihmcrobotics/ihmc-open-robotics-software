package us.ihmc.robotDataVisualizer.logger.converters;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

import us.ihmc.idl.serializers.extra.PropertiesSerializer;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.LogPropertiesPubSubType;
import us.ihmc.robotDataLogger.handshake.ProtoBufferYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;
import us.ihmc.robotDataVisualizer.logger.util.CustomProgressMonitor;
import us.ihmc.tools.compression.SnappyUtils;

public class LogTimeStampedIndexGenerator
{
   private final static String newIndexFile = "robotData_converted.dat";
   
   public static void convert(File logDirectory, LogProperties logProperties) throws IOException
   {
      File handshake = new File(logDirectory, logProperties.getVariables().getHandshakeAsString());
      if (!handshake.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getVariables().getHandshakeAsString());
      }
      DataInputStream handshakeStream = new DataInputStream(new FileInputStream(handshake));
      byte[] handshakeData = new byte[(int) handshake.length()];
      handshakeStream.readFully(handshakeData);
      handshakeStream.close();
      int logLineLength = ProtoBufferYoVariableHandshakeParser.getNumberOfVariables(handshakeData);

      File logdata = new File(logDirectory, logProperties.getVariables().getDataAsString());
      if (!logdata.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getVariables().getDataAsString());
      }

      File index = new File(logDirectory, logProperties.getVariables().getIndexAsString());
      if (!index.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getVariables().getIndexAsString());
      }

      FileInputStream logInputStream = new FileInputStream(logdata);
      FileChannel logChannel = logInputStream.getChannel();

      long[] dataOffsets = readIndexFile(index);
      int[] compressedSizes = calculateCompressedSizes(dataOffsets, logChannel.size());

      int bufferSize = logLineLength * 8;

      ByteBuffer compressedData = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(bufferSize));
      ByteBuffer logLine = ByteBuffer.allocate(bufferSize);

      File indexFile = new File(logDirectory, newIndexFile);
      FileOutputStream indexStream = new FileOutputStream(indexFile);
      FileChannel indexChannel = indexStream.getChannel();
      
      CustomProgressMonitor monitor = new CustomProgressMonitor("Generating timestamps for " + logProperties.getNameAsString(), "Adding timestamps to log index for future functionality improvements.", 0, dataOffsets.length);

      ByteBuffer indexBuffer = ByteBuffer.allocate(16);
      for(int i = 0; i < dataOffsets.length; i++)
      {
         
         monitor.setProgress(i);
         compressedData.clear();
         logLine.clear();
         compressedData.limit(compressedSizes[i]);
         logChannel.read(compressedData);
         compressedData.flip();
         SnappyUtils.uncompress(compressedData, logLine);
         
         long timestamp = logLine.getLong(0);
         indexBuffer.clear();
         indexBuffer.putLong(timestamp);
         indexBuffer.putLong(dataOffsets[i]);
         indexBuffer.flip();
         indexChannel.write(indexBuffer);
      }
      
      monitor.close();
      
      indexChannel.close();
      indexStream.close();
      
      logChannel.close();
      logInputStream.close();
      
      logProperties.getVariables().setTimestamped(true);
      logProperties.getVariables().setIndex(newIndexFile);
      
      File log = new File(logDirectory, YoVariableLoggerListener.propertyFile);
      PropertiesSerializer<LogProperties> writer = new PropertiesSerializer<>(new LogPropertiesPubSubType());
      writer.serialize(log, logProperties);
      
      
      index.delete();
      
   }
   
   
   private static int[] calculateCompressedSizes(long[] dataOffsets, long channelSize) throws IOException
   {
      int[] compressedSizes = new int[dataOffsets.length];
      for (int i = 0; i < dataOffsets.length - 1; i++)
      {
         compressedSizes[i] = (int) (dataOffsets[i + 1] - dataOffsets[i]);
      }
      compressedSizes[dataOffsets.length - 1] = (int) (channelSize - dataOffsets[dataOffsets.length - 1]);
      return compressedSizes;
   }

   private static long[] readIndexFile(File indexData) throws IOException
   {
      @SuppressWarnings("resource")
      FileChannel indexChannel = new FileInputStream(indexData).getChannel();
      long[] dataOffsets = new long[(int) (indexData.length() / 8)];
      ByteBuffer dataOffsetWrap = ByteBuffer.allocateDirect(dataOffsets.length * 8);
      indexChannel.read(dataOffsetWrap);
      dataOffsetWrap.flip();
      dataOffsetWrap.asLongBuffer().get(dataOffsets);
      indexChannel.close();
      
      return dataOffsets;
   }
}
