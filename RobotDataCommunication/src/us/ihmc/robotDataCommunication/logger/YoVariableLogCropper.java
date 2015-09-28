package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

import com.google.common.io.Files;

import us.ihmc.robotDataCommunication.logger.util.CustomProgressMonitor;

public class YoVariableLogCropper extends YoVariableLogReader
{
   private final MultiVideoDataPlayer player;

   private final File model;
   private final File resourceBundle;
   
   protected final File properties;

   public YoVariableLogCropper(MultiVideoDataPlayer player, File logDirectory, LogProperties logProperties)
   {
      super(logDirectory, logProperties);
      this.player = player;
      
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
   }

   public synchronized void crop(File destination, long inStamp, long outStamp)
   {
      CustomProgressMonitor monitor = new CustomProgressMonitor("Cropping data file", "Initializing cropper", 0, 100);

      if (!initialize())
      {
         return;
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

         monitor.setNote("Seeking variable data");
         monitor.setProgress(10);

         File outputFile = new File(destination, logProperties.getVariableDataFile());
         FileOutputStream fileOutputStream = new FileOutputStream(outputFile);
         FileChannel outputChannel = fileOutputStream.getChannel();

         File indexFile = new File(destination, logProperties.getVariablesIndexFile());
         FileOutputStream indexStream = new FileOutputStream(indexFile);
         FileChannel indexChannel = indexStream.getChannel();

         int startPosition = getPosition(inStamp);
         int endPosition = getPosition(outStamp);

         monitor.setNote("Writing variable data");

         ByteBuffer indexBuffer = ByteBuffer.allocateDirect(16);
         for (int i = startPosition; i <= endPosition; i++)
         {
            
            ByteBuffer compressedData = readCompressedData(i);

            indexBuffer.clear();
            indexBuffer.putLong(getTimestamp(i));
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


}
