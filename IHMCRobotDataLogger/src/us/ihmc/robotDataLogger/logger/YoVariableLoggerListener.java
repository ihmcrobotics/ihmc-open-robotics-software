package us.ihmc.robotDataLogger.logger;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.Arrays;

import us.ihmc.commons.PrintTools;
import us.ihmc.idl.serializers.extra.YAMLSerializer;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.CameraAnnouncement;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.rtps.LogParticipantSettings;
import us.ihmc.tools.compression.SnappyUtils;

public class YoVariableLoggerListener implements YoVariablesUpdatedListener
{
   private static final int FLUSH_EVERY_N_PACKETS = 250;
   
   public static final String propertyFile = "robotData.log";
   private static final String handshakeFilename = "handshake.proto";
   private static final String dataFilename = "robotData.bsz";
   private static final String modelFilename = "model.sdf";
   private static final String modelResourceBundle = "resources.zip";
   private static final String indexFilename = "robotData.dat";
   private static final String summaryFilename = "summary.csv";

   private final Object synchronizer = new Object();
   private final Object timestampUpdater = new Object();
   
   private final boolean flushAggressivelyToDisk;
   
   private final File tempDirectory;
   private final File finalDirectory;
   private final YoVariableLoggerOptions options;
   private FileChannel dataChannel;
   private FileChannel indexChannel;

   private final ByteBuffer indexBuffer = ByteBuffer.allocate(16);
   private ByteBuffer compressedBuffer;

   private YoVariableClient yoVariableClient;
   private volatile boolean connected = false;

   private final LogPropertiesWriter logProperties;
   private ArrayList<VideoDataLoggerInterface> videoDataLoggers = new ArrayList<>();

   private final ArrayList<CameraAnnouncement> cameras = new ArrayList<>();

   private boolean clearingLog = false;
   
   private long currentIndex = 0;

   private long lastReceivedTimestamp = Long.MIN_VALUE;
   
   private YoVariableSummarizer yoVariableSummarizer = null;
   
   public YoVariableLoggerListener(File tempDirectory, File finalDirectory, String timestamp, Announcement request, YoVariableLoggerOptions options)
   {
      System.out.println(request);
      this.flushAggressivelyToDisk = options.isFlushAggressivelyToDisk();
      this.tempDirectory = tempDirectory;
      this.finalDirectory = finalDirectory;
      this.options = options;
      logProperties = new LogPropertiesWriter(new File(tempDirectory, propertyFile));
      logProperties.getVariables().setHandshake(handshakeFilename);
      logProperties.getVariables().setData(dataFilename);
      logProperties.getVariables().setCompressed(true);
      logProperties.getVariables().setTimestamped(true);
      logProperties.getVariables().setIndex(indexFilename);

      logProperties.setName(request.getNameAsString());
      logProperties.setTimestamp(timestamp);
      
      if(!options.getDisableVideo())
      {
         
         System.out.println("Cameras: " + request.getCameras());
         cameras.addAll(Arrays.asList(request.getCameras().toArray()));
         
      }
      else
      {
         PrintTools.warn(this, "cameras.yaml not found, disabling video");
      }

   }

   public boolean changesVariables()
   {
      return false;
   }

   private void logHandshake(LogHandshake handshake, YoVariableHandshakeParser handshakeParser)
   {
      File handshakeFile = new File(tempDirectory, handshakeFilename);
      try
      {
         YAMLSerializer<Handshake> serializer = new YAMLSerializer<>(new HandshakePubSubType());
         serializer.serialize(handshakeFile, handshake.getHandshake());
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      if (handshake.getModelLoaderClass() != null)
      {
         logProperties.getModel().setLoader(handshake.getModelLoaderClass());
         logProperties.getModel().setName(handshake.getModelName());
         for(String resourceDirectory : handshake.getResourceDirectories())
         {
            logProperties.getModel().getResourceDirectoriesList().add(resourceDirectory);
         }
         logProperties.getModel().setPath(modelFilename);
         logProperties.getModel().setResourceBundle(modelResourceBundle);

         File modelFile = new File(tempDirectory, modelFilename);
         File resourceFile = new File(tempDirectory, modelResourceBundle);
         try
         {
            FileOutputStream modelStream = new FileOutputStream(modelFile, false);
            modelStream.write(handshake.getModel());
            modelStream.getFD().sync();
            modelStream.close();
            FileOutputStream resourceStream = new FileOutputStream(resourceFile, false);
            resourceStream.write(handshake.getResourceZip());
            resourceStream.getFD().sync();
            resourceStream.close();

         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
      }
      
      if(handshake.getHandshake().getSummary().getCreateSummary())
      {
         yoVariableSummarizer = new YoVariableSummarizer(handshakeParser.getYoVariablesList(), handshake.getHandshake().getSummary().getSummaryTriggerVariableAsString(), handshake.getHandshake().getSummary().getSummarizedVariables().toStringArray());
         logProperties.getVariables().setSummary(summaryFilename);
      }
   }

   public void receivedTimestampAndData(long timestamp, ByteBuffer buffer)
   {
      receivedTimestampOnly(timestamp); // Call from here as backup for the UDP channel.
      
      connected = true;

      synchronized (synchronizer)
      {
         if (!clearingLog && dataChannel != null)
         {
            try
            {
               if(yoVariableSummarizer != null)
               {
                  yoVariableSummarizer.setBuffer(buffer);
               }
               buffer.clear();
               compressedBuffer.clear();
               SnappyUtils.compress(buffer, compressedBuffer);
               compressedBuffer.flip();

               indexBuffer.clear();
               indexBuffer.putLong(timestamp);
               indexBuffer.putLong(dataChannel.position());
               indexBuffer.flip();

               indexChannel.write(indexBuffer);
               dataChannel.write(compressedBuffer);
               
               if(flushAggressivelyToDisk)
               {
                  if(++currentIndex % FLUSH_EVERY_N_PACKETS == 0)
                  {
                     indexChannel.force(false);
                     dataChannel.force(false);
                  }                  
               }
               
               if(yoVariableSummarizer != null)
               {
                  yoVariableSummarizer.update();
               }
            }
            catch (IOException e)
            {
               throw new RuntimeException(e);
            }
         }
      }
   }

   public void disconnected()
   {
      try
      {
         dataChannel.close();
         indexChannel.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      for (VideoDataLoggerInterface videoDataLogger : videoDataLoggers)
      {
         videoDataLogger.close();
      }

      
      if (!connected)
      {
         System.err.println("Never started logging, cleaning up");
         for (VideoDataLoggerInterface videoDataLogger : videoDataLoggers)
         {
            videoDataLogger.removeLogFiles();
         }

         File handshakeFile = new File(tempDirectory, handshakeFilename);
         if (handshakeFile.exists())
         {
            System.out.println("Deleting handshake file");
            handshakeFile.delete();
         }

         File properties = new File(tempDirectory, propertyFile);
         if (properties.exists())
         {
            System.out.println("Deleting properties file");
            properties.delete();
         }

         File model = new File(tempDirectory, modelFilename);
         if (model.exists())
         {
            System.out.println("Deleting model file");
            model.delete();
         }

         File resources = new File(tempDirectory, modelResourceBundle);
         {
            System.out.println("Deleting resource bundle");
            resources.delete();
         }

         File dataFile = new File(tempDirectory, dataFilename);
         if (dataFile.exists())
         {
            System.out.println("Deleting data file");
            dataFile.delete();
         }

         File indexFile = new File(tempDirectory, indexFilename);
         if (indexFile.exists())
         {
            System.out.println("Deleting index file");
            indexFile.delete();
         }

         if (tempDirectory.exists())
         {
            System.out.println("Deleting log directory");
            tempDirectory.delete();
         }

      }
      else
      {

         if(yoVariableSummarizer != null)
         {
            yoVariableSummarizer.writeData(new File(tempDirectory, summaryFilename));
         }

         tempDirectory.renameTo(finalDirectory);
      }
   }

   public void setYoVariableClient(YoVariableClient client)
   {
      this.yoVariableClient = client;
   }

   public void receiveTimedOut()
   {
      if (connected)
      {
         System.out.println("Connection lost, closing client.");
         yoVariableClient.disconnected();
      }
      else
      {
         System.out.println("Cannot connect to client, closing");
         yoVariableClient.disconnected();
      }
   }

   public boolean updateYoVariables()
   {
      return false;
   }

   @Override
   public int getDisplayOneInNPackets()
   {
      return 1;
   }

   @Override
   public void setShowOverheadView(boolean showOverheadView)
   {
   }

   @Override
   public void start(LogHandshake handshake, YoVariableHandshakeParser handshakeParser)
   {
      logHandshake(handshake, handshakeParser);

      int bufferSize = handshakeParser.getBufferSize();
      this.compressedBuffer = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(bufferSize));

      File dataFile = new File(tempDirectory, dataFilename);
      File indexFile = new File(tempDirectory, indexFilename);

      synchronized (synchronizer)
      {
         try
         {
            dataChannel = new FileOutputStream(dataFile, false).getChannel();
            indexChannel = new FileOutputStream(indexFile, false).getChannel();
            
         }
         catch (FileNotFoundException e)
         {
            throw new RuntimeException(e);
         }
      }

      if (!options.getDisableVideo())
      {
         for (CameraAnnouncement camera : cameras)
         {
            try
            {
               switch(camera.getType())
               {
               case CAPTURE_CARD:
                  videoDataLoggers.add(new BlackmagicVideoDataLogger(camera.getNameAsString(), tempDirectory, logProperties, Byte.parseByte(camera.getIdentifierAsString()), options));
                  break;
               case NETWORK_STREAM:
                  videoDataLoggers.add(new NetworkStreamVideoDataLogger(tempDirectory, logProperties, LogParticipantSettings.domain, camera.getIdentifierAsString()));
                  break;
               }
            }
            catch (IOException e)
            {
               System.err.println("Cannot start video data logger");
               e.printStackTrace();
            }
         }
      }
      
      // Write data to file, force it to exist
      try
      {
         logProperties.store();

         dataChannel.force(true);
         indexChannel.force(true);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public void receivedTimestampOnly(long timestamp)
   {
      synchronized(timestampUpdater)
      {
         if(timestamp > lastReceivedTimestamp) // Check if this a newer timestamp. UDP is out of order and the TCP packets also call this function
         {
            for (int i = 0; i < videoDataLoggers.size(); i++)
            {
               videoDataLoggers.get(i).timestampChanged(timestamp);
            }
         }
      }
   }

   @Override
   public void clearLog(String guid)
   {
      synchronized (synchronizer)
      {
         clearingLog = true;
      }
      try
      {
         System.out.println("Clearing log.");
         dataChannel.truncate(0);
         indexChannel.truncate(0);
         for (VideoDataLoggerInterface videoDataLogger : videoDataLoggers)
         {
            videoDataLogger.restart();
         }
         
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      if(yoVariableSummarizer != null)
      {
         yoVariableSummarizer.restart();
      }
      synchronized (synchronizer)
      {
         clearingLog = false;
      }
   }

   @Override
   public boolean executeVariableChangedListeners()
   {
      return false;
   }
}
