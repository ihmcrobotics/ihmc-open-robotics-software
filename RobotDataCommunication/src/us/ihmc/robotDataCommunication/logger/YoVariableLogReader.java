package us.ihmc.robotDataCommunication.logger;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

import us.ihmc.robotDataCommunication.LogIndex;
import us.ihmc.robotDataCommunication.YoVariableHandshakeParser;
import us.ihmc.tools.compression.SnappyUtils;

public class YoVariableLogReader
{

   private boolean initialized = false;
   private final File logDirectory;
   protected final LogProperties logProperties;
   
   protected final File handshake;
   private FileChannel logChannel;
   private LogIndex logIndex;
   private ByteBuffer compressedData;
   private ByteBuffer uncompressedData;
   private FileInputStream logInputStream;

   public YoVariableLogReader(File logDirectory, LogProperties logProperties)
   {

      this.logDirectory = logDirectory;
      this.logProperties = logProperties;


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
            int logLineLength = YoVariableHandshakeParser.getNumberOfVariables(handshakeData);

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
            
            initialized = true;
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }  
      }
      return initialized;
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

}