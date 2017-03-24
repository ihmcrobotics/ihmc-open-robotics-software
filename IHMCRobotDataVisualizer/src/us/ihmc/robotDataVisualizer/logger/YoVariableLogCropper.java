package us.ihmc.robotDataVisualizer.logger;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.logger.YoVariableLogReader;
import us.ihmc.robotDataVisualizer.logger.util.CustomProgressMonitor;
import us.ihmc.robotDataVisualizer.logger.util.ProgressMonitorInterface;

public class YoVariableLogCropper extends YoVariableLogReader
{
   private final MultiVideoDataPlayer player;


   

   public YoVariableLogCropper(MultiVideoDataPlayer player, File logDirectory, LogProperties logProperties)
   {
      super(logDirectory, logProperties);
      this.player = player;
      
      
   }

   public synchronized void crop(File destination, long inStamp, long outStamp)
   {
      ProgressMonitorInterface monitor = new CustomProgressMonitor("Cropping data file", "Initializing cropper", 0, 100);

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
         copyMetaData(destination);

         monitor.setNote("Seeking variable data");
         monitor.setProgress(10);

         File outputFile = new File(destination, logProperties.getVariables().getDataAsString());
         FileOutputStream fileOutputStream = new FileOutputStream(outputFile);
         FileChannel outputChannel = fileOutputStream.getChannel();

         File indexFile = new File(destination, logProperties.getVariables().getIndexAsString());
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
