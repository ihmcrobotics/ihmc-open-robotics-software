package us.ihmc.robotDataLogger.logger;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

import com.google.common.io.Files;

import us.ihmc.robotDataLogger.LogIndex;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.ProtoBufferYoVariableHandshakeParser;
import us.ihmc.tools.compression.SnappyUtils;

public class YoVariableLogReader
{

   private boolean initialized = false;
   protected final File logDirectory;
   protected final LogProperties logProperties;
   
   
   private int logLineLength;
   private int numberOfEntries;
   
   
   protected final File handshake;
   private FileChannel logChannel;
   private LogIndex logIndex;
   private ByteBuffer compressedData;
   private ByteBuffer uncompressedData;
   private FileInputStream logInputStream;
   
   protected final File properties;
   private final File model;
   private final File resourceBundle;
   private final File summary;

   public YoVariableLogReader(File logDirectory, LogProperties logProperties)
   {

      this.logDirectory = logDirectory;
      this.logProperties = logProperties;

      
      properties = new File(logDirectory, YoVariableLoggerListener.propertyFile);

      if (!logProperties.getModel().getPathAsString().isEmpty())
      {
         model = new File(logDirectory, logProperties.getModel().getPathAsString());
      }
      else
      {
         model = null;
      }

      if (!logProperties.getModel().getResourceBundleAsString().isEmpty())
      {
         resourceBundle = new File(logDirectory, logProperties.getModel().getResourceBundleAsString());
      }
      else
      {
         resourceBundle = null;
      }
      
      if(!logProperties.getVariables().getSummaryAsString().isEmpty())
      {
         summary = new File(logDirectory, logProperties.getVariables().getSummaryAsString());
      }
      else
      {
         summary = null;
      }

      handshake = new File(logDirectory, logProperties.getVariables().getHandshakeAsString());
      if (!handshake.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getVariables().getHandshakeAsString());
      }

   }

   protected boolean initialize()
   {
      if (!initialized)
      {
         try
         {
            DataInputStream handshakeStream = new DataInputStream(new FileInputStream(handshake));
            byte[] handshakeData = new byte[(int) handshake.length()];
            handshakeStream.readFully(handshakeData);
            handshakeStream.close();
            logLineLength = ProtoBufferYoVariableHandshakeParser.getNumberOfVariables(handshakeData);

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

            logInputStream = new FileInputStream(logdata);
            logChannel = logInputStream.getChannel();

            logIndex = new LogIndex(index, logChannel.size());
            int bufferSize = logLineLength * 8;
            compressedData = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(bufferSize));
            uncompressedData = ByteBuffer.allocate(bufferSize);
            
            numberOfEntries = logIndex.getNumberOfEntries();
            initialized = true;
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }  
      }
      return initialized;
   }

   public int getNumberOfVariables()
   {
      return logLineLength;
   }
   
   public int getNumberOfEntries()
   {
      return numberOfEntries;
   }
   
   public void close()
   {
      try
      {
         logChannel.close();
         logInputStream.close();
      }
      catch (IOException e)
      {
         // Nothing to do here
      }
   }

   protected int getPosition(long timestamp) throws IOException
   {
      return logIndex.seek(timestamp);
   }
   
   protected long getDataOffset(int position)
   {
      return logIndex.dataOffsets[position];
   }
   
   protected int getCompressedSize(int position)
   {
      return logIndex.compressedSizes[position];
   }
   
   protected long getTimestamp(int position)
   {
      return logIndex.timestamps[position];
   }
   
   protected ByteBuffer readCompressedData(int position) throws IOException
   {
      int size = getCompressedSize(position);
      long startOffset = getDataOffset(position);
      logChannel.position(startOffset);
      compressedData.clear();
      compressedData.limit(size);
      logChannel.read(compressedData);
      compressedData.flip();
      
      return compressedData;
      
   }
   
   protected ByteBuffer readData(int position) throws IOException
   {
      ByteBuffer compressedData = readCompressedData(position);
      uncompressedData.clear();
      SnappyUtils.uncompress(compressedData, uncompressedData);
      uncompressedData.flip();
      return uncompressedData;
   }

   protected void copyMetaData(File destination) throws IOException
   {
      File propertiesDestination = new File(destination, YoVariableLoggerListener.propertyFile);
      Files.copy(properties, propertiesDestination);
   
      File handShakeDestination = new File(destination, logProperties.getVariables().getHandshakeAsString());
      Files.copy(handshake, handShakeDestination);
   
      if (model != null)
      {
         File modelDesitination = new File(destination, logProperties.getModel().getPathAsString());
         Files.copy(model, modelDesitination);
      }
      if (resourceBundle != null)
      {
         File resourceDestination = new File(destination, logProperties.getModel().getResourceBundleAsString());
         Files.copy(resourceBundle, resourceDestination);
      }
      
      if(summary != null)
      {
         File summaryDestination = new File(destination, logProperties.getVariables().getSummaryAsString());
         Files.copy(summary, summaryDestination);
      }
   }

}