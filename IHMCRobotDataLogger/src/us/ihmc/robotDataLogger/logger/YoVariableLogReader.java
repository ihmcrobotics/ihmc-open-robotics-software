package us.ihmc.robotDataLogger.logger;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

import com.google.common.io.Files;

import us.ihmc.robotDataLogger.LogIndex;
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

      if (logProperties.getModelPath() != null)
      {
         model = new File(logDirectory, logProperties.getModelPath());
      }
      else
      {
         model = null;
      }

      if (logProperties.getModelResourceBundlePath() != null)
      {
         resourceBundle = new File(logDirectory, logProperties.getModelResourceBundlePath());
      }
      else
      {
         resourceBundle = null;
      }
      
      if(logProperties.getSummaryFile() != null)
      {
         summary = new File(logDirectory, logProperties.getSummaryFile());
      }
      else
      {
         summary = null;
      }

      handshake = new File(logDirectory, logProperties.getHandshakeFile());
      if (!handshake.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getHandshakeFile());
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

            File logdata = new File(logDirectory, logProperties.getVariableDataFile());
            if (!logdata.exists())
            {
               throw new RuntimeException("Cannot find " + logProperties.getVariableDataFile());
            }

            File index = new File(logDirectory, logProperties.getVariablesIndexFile());
            if (!index.exists())
            {
               throw new RuntimeException("Cannot find " + logProperties.getVariablesIndexFile());
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
   
      File handShakeDestination = new File(destination, logProperties.getHandshakeFile());
      Files.copy(handshake, handShakeDestination);
   
      if (model != null)
      {
         File modelDesitination = new File(destination, logProperties.getModelPath());
         Files.copy(model, modelDesitination);
      }
      if (resourceBundle != null)
      {
         File resourceDestination = new File(destination, logProperties.getModelResourceBundlePath());
         Files.copy(resourceBundle, resourceDestination);
      }
      
      if(summary != null)
      {
         File summaryDestination = new File(destination, logProperties.getSummaryFile());
         Files.copy(summary, summaryDestination);
      }
   }

}