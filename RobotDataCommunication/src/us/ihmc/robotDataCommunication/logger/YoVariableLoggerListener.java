package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.YoVariablesUpdatedListener;
import us.ihmc.robotDataCommunication.generated.YoProtoHandshakeProto.YoProtoHandshake;
import us.ihmc.robotDataCommunication.jointState.JointState;

import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class YoVariableLoggerListener implements YoVariablesUpdatedListener
{
   public static final String propertyFile = "robotData.log";
   private static final long connectTimeout = 30000;
   private static final long disconnectTimeout = 5000; 
   private static final String handshakeFilename = "handshake.proto";
   private static final String dataFilename = "robotData.bin";
   
   private final Object synchronizer = new Object();
   
   private final File directory;
   private final YoVariableLoggerOptions options;
   private FileChannel dataChannel;
   
   private YoVariableClient yoVariableClient;
   private boolean connected = false;
   private long totalTimeout = 0;
  
   private final LogPropertiesWriter logProperties;
   private ArrayList<VideoDataLogger> videoDataLoggers = new ArrayList<VideoDataLogger>();
   
   public YoVariableLoggerListener(File directory, YoVariableLoggerOptions options)
   {
      this.directory = directory; 
      this.options = options;
      logProperties = new LogPropertiesWriter(new File(directory, propertyFile));
      logProperties.setHandshakeFile(handshakeFilename);
      logProperties.setVariableDataFile(dataFilename);
     
   }

   public boolean changesVariables()
   {
      return false;
   }

   public void setRegistry(YoVariableRegistry registry)
   {
      
   }

   public void registerDynamicGraphicObjectListsRegistry(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      
   }

   public void receivedHandshake(YoProtoHandshake handshake)
   {
      File handshakeFile = new File(directory, handshakeFilename);
      try
      {
         FileOutputStream handshakeStream = new FileOutputStream(handshakeFile, false);
         handshakeStream.write(handshake.toByteArray());
         handshakeStream.close();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      
   }

   public void receivedUpdate(long timestamp, ByteBuffer buffer)
   {
      connected = true;
      totalTimeout = 0;
      for(VideoDataLogger videoDataLogger : videoDataLoggers)
      {
         videoDataLogger.timestampChanged(timestamp);
      }
      
      synchronized (synchronizer)
      {
         if(dataChannel != null)
         {
            try
            {
               buffer.clear();
               dataChannel.write(buffer);
            }
            catch (IOException e)
            {
               throw new RuntimeException(e);
            }
         }         
      }
   }

   public void start()
   {
      File dataFile = new File(directory, dataFilename);
      synchronized (synchronizer)
      {
         try
         {
            dataChannel = new FileOutputStream(dataFile, false).getChannel();
         }
         catch (FileNotFoundException e)
         {
            throw new RuntimeException(e);
         }         
      }
      
      if(!options.getDisableVideo())
      {
         try
         {
            
            videoDataLoggers.add(new VideoDataLogger(directory, logProperties, VideoSettings.SONY_720P60_TRIPOD, options));
            videoDataLoggers.add(new VideoDataLogger(directory, logProperties, VideoSettings.BLACKMAGIC_1080P30_CRANE, options));
         }
         catch (IOException e)
         {
            System.err.println("Cannot start video data logger");
            e.printStackTrace();
         }   
      }
   }

   public void disconnected()
   {
      try
      {
         dataChannel.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      
      for(VideoDataLogger videoDataLogger : videoDataLoggers)
      {
         videoDataLogger.close();
      }
      
      try
      {
         logProperties.store();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void setJointStates(List<JointState<? extends Joint>> jointStates)
   {
      
   }

   public void setYoVariableClient(YoVariableClient client)
   {
      this.yoVariableClient = client;
   }

   public void receiveTimedOut(long timeoutInMillis)
   {
      totalTimeout += timeoutInMillis;
      if(connected)
      {
         if(totalTimeout > disconnectTimeout)
         {
            System.out.println("Timeout reached: " + totalTimeout + ". Connection lost, closing client.");
            yoVariableClient.close();
         }
      }
      else
      {
         if(totalTimeout > connectTimeout)
         {
            System.out.println("Cannot connect to client, closing");
            yoVariableClient.close();
         }
      }
   }

   public boolean populateRegistry()
   {
      return false;
   }

}
