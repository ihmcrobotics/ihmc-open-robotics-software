package us.ihmc.robotDataCommunication.logger;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

import us.ihmc.robotDataCommunication.LogIndex;
import us.ihmc.robotDataCommunication.YoVariableHandshakeParser;
import us.ihmc.utilities.compression.SnappyUtils;
import us.ihmc.utilities.gui.CustomProgressMonitor;

import com.google.common.io.Files;

public class YoVariableLogCropper
{
   private boolean initialized = false;

   private final File logDirectory;
   private final LogProperties logProperties;
   private final MultiVideoDataPlayer player;

   private final File properties;
   private final File handshake;
   private final File model;
   private final File resourceBundle;

   private FileChannel logChannel;

   private LogIndex logIndex;

   private ByteBuffer compressedData;
   private FileInputStream logInputStream;

   public YoVariableLogCropper(MultiVideoDataPlayer player, File logDirectory, LogProperties logProperties) throws IOException
   {
      this.logDirectory = logDirectory;
      this.logProperties = logProperties;
      this.player = player;

      properties = new File(logDirectory, YoVariableLoggerListener.propertyFile);
      handshake = new File(logDirectory, logProperties.getHandshakeFile());
      if (!handshake.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getHandshakeFile());
      }

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
   }

   private void initialize(File logDirectory, LogProperties logProperties) throws FileNotFoundException, IOException
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

      initialized = true;
   }

   public synchronized void crop(File destination, long inStamp, long outStamp)
   {
      CustomProgressMonitor monitor = new CustomProgressMonitor("Cropping data file", "Initializing cropper", 0, 100);
      if (!initialized)
      {
         try
         {
            initialize(logDirectory, logProperties);
         }
         catch (IOException e)
         {
            e.printStackTrace();
            return;
         }
      }

      try
      {
         monitor.setNote("Creating directories");
         monitor.setProgress(3);
         if (destination.exists())
         {
            if (!destination.isDirectory())
            {
               monitor.setProgress(0);
               monitor.setError("Destination " + destination.getAbsolutePath() + " already exists.");
               return;
            }
            else if (destination.list().length > 0)
            {
               monitor.setProgress(0);
               monitor.setError("Destination " + destination.getAbsolutePath() + " is not empty.");
               return;
            }
         }
         else if (!destination.mkdir())
         {
            monitor.setProgress(0);
            monitor.setError("Cannot make directory " + destination.getAbsolutePath());
            return;
         }

         monitor.setNote("Copying description files");
         monitor.setProgress(4);
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

         logChannel.position(0);

         monitor.setNote("Seeking variable data");
         monitor.setProgress(10);

         File outputFile = new File(destination, logProperties.getVariableDataFile());
         FileOutputStream fileOutputStream = new FileOutputStream(outputFile);
         FileChannel outputChannel = fileOutputStream.getChannel();
         
         File indexFile = new File(destination, logProperties.getVariablesIndexFile());
         FileOutputStream indexStream = new FileOutputStream(indexFile);
         FileChannel indexChannel = indexStream.getChannel();

         int startPosition = logIndex.seek(inStamp);
         int endPosition = logIndex.seek(outStamp);


         monitor.setNote("Writing variable data");
         long startOffset = logIndex.dataOffsets[startPosition];
         logChannel.position(startOffset);

         ByteBuffer indexBuffer = ByteBuffer.allocateDirect(16);
         for(int i = startPosition; i <= endPosition; i++)
         {
            int size = logIndex.compressedSizes[i];
            compressedData.clear();
            compressedData.limit(size);
            logChannel.read(compressedData);
            compressedData.flip();
            
            indexBuffer.clear();
            indexBuffer.putLong(logIndex.timestamps[i]);
            indexBuffer.putLong(outputChannel.position());
            indexBuffer.flip();
            indexChannel.write(indexBuffer);
            
            outputChannel.write(compressedData);
         }

         outputChannel.close();
         fileOutputStream.close();
         
         indexChannel.close();
         indexStream.close();

         monitor.setNote("Cropping video files");

         if (player != null)
            player.crop(destination, inStamp, outStamp, monitor);
         monitor.close();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
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
}
