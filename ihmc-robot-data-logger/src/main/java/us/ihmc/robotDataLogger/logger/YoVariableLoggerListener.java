package us.ihmc.robotDataLogger.logger;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.function.Consumer;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.serializers.extra.YAMLSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.CameraAnnouncement;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakeFileType;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.YoVariableClientInterface;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.rtps.LogParticipantSettings;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.tools.compression.SnappyUtils;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableLoggerListener implements YoVariablesUpdatedListener
{
   private static final int FLUSH_EVERY_N_PACKETS = 250;
   public static final long STATUS_PACKET_RATE = Conversions.secondsToNanoseconds(5.0);
   private static final long VIDEO_RECORDING_TIMEOUT = Conversions.secondsToNanoseconds(1.0);

   public static final String propertyFile = "robotData.log";
   private static final String handshakeFilename = "handshake.yaml";
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

   private volatile boolean connected = false;

   private final LogPropertiesWriter logProperties;
   private ArrayList<VideoDataLoggerInterface> videoDataLoggers = new ArrayList<>();

   private final ArrayList<CameraAnnouncement> cameras = new ArrayList<>();

   private boolean clearingLog = false;

   private long currentIndex = 0;

   private long lastReceivedTimestamp = Long.MIN_VALUE;

   private final Announcement request;
   private final Consumer<Announcement> doneListener;

   private YoVariableSummarizer yoVariableSummarizer = null;

   // Reconstruction variables for disk data format
   private List<YoVariable<?>> variables;
   private List<JointState> jointStates;
   private ByteBuffer dataBuffer;
   private LongBuffer dataBufferAsLong;

   private YoVariableClientInterface yoVariableClientInterface = null;

   private long lastStatusUpdateTimestamp = 0;
   private long logStartedTimestamp = 0;

   public YoVariableLoggerListener(File tempDirectory, File finalDirectory, String timestamp, Announcement request, YoVariableLoggerOptions options,
                                   Consumer<Announcement> doneListener)
   {
      LogTools.info(toString(request));
      this.flushAggressivelyToDisk = options.isFlushAggressivelyToDisk();
      this.tempDirectory = tempDirectory;
      this.finalDirectory = finalDirectory;
      this.options = options;
      this.request = request;
      this.doneListener = doneListener;
      logProperties = new LogPropertiesWriter(new File(tempDirectory, propertyFile));
      logProperties.getVariables().setHandshake(handshakeFilename);
      logProperties.getVariables().setData(dataFilename);
      logProperties.getVariables().setCompressed(true);
      logProperties.getVariables().setTimestamped(true);
      logProperties.getVariables().setIndex(indexFilename);
      logProperties.getVariables().setHandshakeFileType(HandshakeFileType.IDL_YAML);

      logProperties.setName(request.getNameAsString());
      logProperties.setTimestamp(timestamp);

      if (!options.getDisableVideo())
      {
         IDLSequence.Object<CameraAnnouncement> cameras = request.getCameras();
         
         StringBuilder builder = new StringBuilder();
         builder.append("Cameras:");
         for (int i = 0; i < cameras.size(); i++)
         {
            this.cameras.add(cameras.get(i));
            builder.append("\n" + toString(cameras.get(i)));
         }

         LogTools.info(builder.toString());
      }
      else
      {
         LogTools.warn("Video capture disabled. Ignoring camera's and network streams");
      }

   }

   @Override
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
         for (String resourceDirectory : handshake.getResourceDirectories())
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

      if (handshake.getHandshake().getSummary().getCreateSummary())
      {
         yoVariableSummarizer = new YoVariableSummarizer(handshakeParser.getYoVariablesList(),
                                                         handshake.getHandshake().getSummary().getSummaryTriggerVariableAsString(),
                                                         handshake.getHandshake().getSummary().getSummarizedVariables().toStringArray());
         logProperties.getVariables().setSummary(summaryFilename);
      }
   }

   @Override
   public void receivedTimestampAndData(long timestamp)
   {
      receivedTimestampOnly(timestamp); // Call from here as backup for the UDP channel.

      ByteBuffer buffer = reconstructBuffer(timestamp);

      connected = true;

      synchronized (synchronizer)
      {
         if (!clearingLog && dataChannel != null)
         {
            try
            {
               if (yoVariableSummarizer != null)
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

               if (flushAggressivelyToDisk)
               {
                  if (++currentIndex % FLUSH_EVERY_N_PACKETS == 0)
                  {
                     indexChannel.force(false);
                     dataChannel.force(false);
                  }
               }

               if (yoVariableSummarizer != null)
               {
                  yoVariableSummarizer.update();
               }

               updateStatus();
            }
            catch (IOException e)
            {
               throw new RuntimeException(e);
            }
         }

      }

   }

   private void updateStatus()
   {
      synchronized (synchronizer)
      {
         if (yoVariableClientInterface != null)
         {
            long now = System.nanoTime();

            if (now > (lastStatusUpdateTimestamp + STATUS_PACKET_RATE))
            {

               boolean recordingVideo = false;
               for (int i = 0; i < videoDataLoggers.size(); i++)
               {
                  if ((videoDataLoggers.get(i).getLastFrameReceivedTimestamp() + VIDEO_RECORDING_TIMEOUT) > now)
                  {
                     recordingVideo = true;
                  }
               }

               int time = (int) MathTools.clamp(Conversions.nanosecondsToSeconds(now - logStartedTimestamp), 0, DataServerCommand.getMaximumArgumentValue());
               if (recordingVideo)
               {
                  yoVariableClientInterface.sendCommand(DataServerCommand.LOG_ACTIVE_WITH_CAMERA, time);
               }
               else
               {
                  yoVariableClientInterface.sendCommand(DataServerCommand.LOG_ACTIVE, time);
               }

               lastStatusUpdateTimestamp = now;
            }
         }
      }
   }

   private ByteBuffer reconstructBuffer(long timestamp)
   {
      dataBuffer.clear();
      dataBufferAsLong.clear();

      dataBufferAsLong.put(timestamp);
      for (int i = 0; i < variables.size(); i++)
      {
         dataBufferAsLong.put(variables.get(i).getValueAsLongBits());
      }

      for (int i = 0; i < jointStates.size(); i++)
      {
         jointStates.get(i).get(dataBufferAsLong);
      }

      dataBufferAsLong.flip();
      dataBuffer.clear();
      return dataBuffer;
   }

   @Override
   public void disconnected()
   {
      LogTools.info("Logger disconnected from " + request.getHostNameAsString());

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
         closeVideo(videoDataLogger);
      }

      if (!connected)
      {
         LogTools.error("Never started logging, cleaning up");
         for (VideoDataLoggerInterface videoDataLogger : videoDataLoggers)
         {
            videoDataLogger.removeLogFiles();
         }

         File handshakeFile = new File(tempDirectory, handshakeFilename);
         if (handshakeFile.exists())
         {
            LogTools.info("Deleting handshake file");
            handshakeFile.delete();
         }

         File properties = new File(tempDirectory, propertyFile);
         if (properties.exists())
         {
            LogTools.info("Deleting properties file");
            properties.delete();
         }

         File model = new File(tempDirectory, modelFilename);
         if (model.exists())
         {
            LogTools.info("Deleting model file");
            model.delete();
         }

         File resources = new File(tempDirectory, modelResourceBundle);
         {
            LogTools.info("Deleting resource bundle");
            resources.delete();
         }

         File dataFile = new File(tempDirectory, dataFilename);
         if (dataFile.exists())
         {
            LogTools.info("Deleting data file");
            dataFile.delete();
         }

         File indexFile = new File(tempDirectory, indexFilename);
         if (indexFile.exists())
         {
            LogTools.info("Deleting index file");
            indexFile.delete();
         }

         if (tempDirectory.exists())
         {
            LogTools.info("Deleting log directory");
            tempDirectory.delete();
         }
      }
      else
      {
         if (yoVariableSummarizer != null)
         {
            yoVariableSummarizer.writeData(new File(tempDirectory, summaryFilename));
         }

         tempDirectory.renameTo(finalDirectory);

         doneListener.accept(request);
      }
   }

   private final ExecutorService executor = Executors.newCachedThreadPool();
   private void closeVideo(VideoDataLoggerInterface videoDataLogger)
   {
      Future<?> future = executor.submit(() -> videoDataLogger.close());
      try
      {
         future.get(5, TimeUnit.SECONDS);
      }
      catch (InterruptedException | ExecutionException e)
      {
         e.printStackTrace();
      }
      catch (TimeoutException e)
      {
         LogTools.info("Closing video stream timed out after 5s.");
      }
   }

   @Override
   public boolean updateYoVariables()
   {
      return false;
   }

   @Override
   public void setShowOverheadView(boolean showOverheadView)
   {
   }

   @SuppressWarnings("resource")
   @Override
   public void start(YoVariableClientInterface yoVariableClientInterface, LogHandshake handshake, YoVariableHandshakeParser handshakeParser)
   {
      logHandshake(handshake, handshakeParser);

      int bufferSize = handshakeParser.getBufferSize();
      this.compressedBuffer = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(bufferSize));

      // Initialize disk format variables
      this.dataBuffer = ByteBuffer.allocate(bufferSize);
      this.dataBufferAsLong = dataBuffer.asLongBuffer();
      this.variables = handshakeParser.getYoVariablesList();
      this.jointStates = handshakeParser.getJointStates();

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

         if (!options.getDisableVideo())
         {
            for (CameraAnnouncement camera : cameras)
            {
               try
               {
                  switch (camera.getType())
                  {
                  case CAPTURE_CARD:
                     videoDataLoggers.add(new BlackmagicVideoDataLogger(camera.getNameAsString(), tempDirectory, logProperties,
                                                                        Byte.parseByte(camera.getIdentifierAsString()), options));
                     break;
                  case NETWORK_STREAM:
                     videoDataLoggers.add(new NetworkStreamVideoDataLogger(tempDirectory, logProperties, LogParticipantSettings.videoDomain,
                                                                           camera.getIdentifierAsString()));
                     break;
                  }
               }
               catch (IOException e)
               {
                  LogTools.error("Cannot start video data logger");
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

         this.yoVariableClientInterface = yoVariableClientInterface;

         logStartedTimestamp = System.nanoTime();
      }

   }

   @Override
   public void receivedTimestampOnly(long timestamp)
   {
      synchronized (timestampUpdater)
      {
         if (timestamp > lastReceivedTimestamp) // Check if this a newer timestamp. UDP is out of order and the TCP packets also call this function
         {
            for (int i = 0; i < videoDataLoggers.size(); i++)
            {
               videoDataLoggers.get(i).timestampChanged(timestamp);
            }
            lastReceivedTimestamp = timestamp;
         }
      }
   }

   private void clearLog()
   {
      synchronized (synchronizer)
      {
         clearingLog = true;
      }
      try
      {
         LogTools.info("Clearing log.");
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
      if (yoVariableSummarizer != null)
      {
         yoVariableSummarizer.restart();
      }
      synchronized (synchronizer)
      {
         clearingLog = false;

         logStartedTimestamp = System.nanoTime();
      }
   }

   @Override
   public void connected()
   {

   }

   @Override
   public void receivedCommand(DataServerCommand command, int argument)
   {
      if (command == DataServerCommand.CLEAR_LOG)
      {
         clearLog();
      }
   }

   private static String toString(Announcement announcement)
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Announcement {");
      builder.append("\n  identifier = ");
      builder.append(announcement.identifier_);
      builder.append("\n  name = ");
      builder.append(announcement.name_);
      builder.append("\n  hostName = ");
      builder.append(announcement.hostName_);
      builder.append("\n  log = ");
      builder.append(announcement.log_);
      builder.append("\n}");

      return builder.toString();
   }

   private static String toString(CameraAnnouncement camera)
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CameraAnnouncement {");
      builder.append("\n  type = ");
      builder.append(camera.type_);
      builder.append("\n  name = ");
      builder.append(camera.name_);
      builder.append("\n  identifier = ");
      builder.append(camera.identifier_);
      builder.append("\n}");

      return builder.toString();
   }
}
