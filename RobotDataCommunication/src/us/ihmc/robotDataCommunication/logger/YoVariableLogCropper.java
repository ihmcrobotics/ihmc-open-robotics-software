package us.ihmc.robotDataCommunication.logger;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

import us.ihmc.robotDataCommunication.YoVariableHandshakeParser;
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
   
   private FileChannel logChannel;
   private ByteBuffer logLine;
   private FileInputStream logInputStream;
   
   public YoVariableLogCropper(MultiVideoDataPlayer player, File logDirectory, LogProperties logProperties) throws IOException   
   {      
      this.logDirectory = logDirectory;
      this.logProperties = logProperties;
      this.player = player;
      
      properties = new File(logDirectory, YoVariableLoggerListener.propertyFile);
      handshake = new File(logDirectory, logProperties.getHandshakeFile());
      if(!handshake.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getHandshakeFile());
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
      if(!logdata.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getVariableDataFile());
      }

      
      logInputStream = new FileInputStream(logdata);
      logChannel = logInputStream.getChannel();
      
      int bufferSize = logLineLength * 8;

      logLine = ByteBuffer.allocate(bufferSize);
      
      initialized = true;
   }
   
   public synchronized void crop(File destination, long inStamp, long outStamp)
   {
      CustomProgressMonitor monitor = new CustomProgressMonitor("Cropping data file", "Initializing cropper", 0, 100);
      if(!initialized)
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
         if(destination.exists())
         {
            System.err.println("Destination directory " + destination.getAbsolutePath() + " already exists");
            return;
         }
         
         if(!destination.mkdir())
         {
            System.err.println("Cannot make directory " + destination.getAbsolutePath());
            return;
         }
         
         monitor.setNote("Copying description files");
         monitor.setProgress(4);
         File propertiesDestination = new File(destination, YoVariableLoggerListener.propertyFile);
         Files.copy(properties, propertiesDestination);
         
         File handShakeDestination = new File(destination, logProperties.getHandshakeFile()); 
         Files.copy(handshake, handShakeDestination);
         
         logChannel.position(0);
         
         monitor.setNote("Seeking variable data");
         monitor.setProgress(10);
         
         File outputFile = new File(destination, logProperties.getVariableDataFile());
         FileOutputStream fileOutputStream = new FileOutputStream(outputFile);
         FileChannel outputChannel = fileOutputStream.getChannel();
         
         ByteBuffer timestampBuffer = ByteBuffer.allocate(8);
         
         long position = 0;
         long firstTimestamp = Long.MIN_VALUE;
         while(logChannel.read(timestampBuffer) > 0)
         {
            logChannel.position(position);
            long timestamp = timestampBuffer.getLong(0);
            
            if(firstTimestamp == Long.MIN_VALUE)
            {
               firstTimestamp = timestamp;
            }
            
            monitor.setProgress(10 + ((int) (10.0 * ((double)(timestamp - firstTimestamp))/((double)inStamp - firstTimestamp))));
            
            if(timestamp >= inStamp)
            {
               logChannel.position(position);
               break;
            }
            position += logLine.capacity();
            timestampBuffer.clear();
         }
         monitor.setNote("Writing variable data");
         
         logLine.clear();
         while((logChannel.read(logLine)) > 0)
         {
            long timestamp = logLine.getLong(0);
            if(timestamp >= inStamp && timestamp <= outStamp)
            {
               logLine.rewind();
               outputChannel.write(logLine);
               
               monitor.setProgress(20 + ((int) (30.0 * ((double)(timestamp - inStamp))/((double)(outStamp - inStamp)))));
            }
            else if (timestamp > outStamp)
            {
               break;
            }
            logLine.clear();
         }
         
         
         outputChannel.close();
         fileOutputStream.close();
         
         monitor.setNote("Cropping video files");
         
         player.crop(destination, inStamp, outStamp, monitor);
         monitor.close();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      finally
      {
         
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
