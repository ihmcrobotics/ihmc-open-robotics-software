package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.multicastLogDataProtocol.LogUtils;
import us.ihmc.multicastLogDataProtocol.broadcast.AnnounceRequest;
import us.ihmc.multicastLogDataProtocol.control.LogHandshake;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelLoader;
import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.YoVariablesUpdatedListener;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.robotDataCommunication.logger.util.CookieJar;
import us.ihmc.robotDataCommunication.logger.util.PipedCommandExecutor;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.utilities.compression.SnappyUtils;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class YoVariableLoggerListener implements YoVariablesUpdatedListener
{
   public static final String propertyFile = "robotData.log";
   private static final String handshakeFilename = "handshake.proto";
   private static final String dataFilename = "robotData.bsz";
   private static final String modelFilename = "model.sdf";
   private static final String modelResourceBundle = "resources.zip";
   private static final String indexFilename = "robotData.dat";

   private final Object synchronizer = new Object();

   private final File tempDirectory;
   private final File finalDirectory;
   private final YoVariableLoggerOptions options;
   private FileChannel dataChannel;
   private FileChannel indexChannel;

   private final AnnounceRequest request;

   private final ByteBuffer indexBuffer = ByteBuffer.allocate(16);
   private ByteBuffer compressedBuffer;

   private YoVariableClient yoVariableClient;
   private volatile boolean connected = false;

   private final LogPropertiesWriter logProperties;
   private ArrayList<VideoDataLoggerInterface> videoDataLoggers = new ArrayList<VideoDataLoggerInterface>();

   private final ArrayList<VideoSettings> cameras = new ArrayList<>();
   private final InetSocketAddress videoStreamAddress;

   private boolean clearingLog = false;

   public YoVariableLoggerListener(File tempDirectory, File finalDirectory, String timestamp, AnnounceRequest request, YoVariableLoggerOptions options)
   {
      System.out.println(request);
      this.tempDirectory = tempDirectory;
      this.finalDirectory = finalDirectory;
      this.options = options;
      this.request = request;
      logProperties = new LogPropertiesWriter(new File(tempDirectory, propertyFile));
      logProperties.setHandshakeFile(handshakeFilename);
      logProperties.setVariableDataFile(dataFilename);
      logProperties.setCompressed(true);
      logProperties.setTimestampedIndex(true);
      logProperties.setVariablesIndexFile(indexFilename);

      logProperties.setLogName(request.getName());
      logProperties.setTimestamp(timestamp);
      try
      {
         FileInputStream configuration = new FileInputStream(options.getConfigurationFile());
         List<VideoSettings> cameraSettings = VideoSettings.loadCameraSettings(configuration);

         System.out.println("Cameras: " + Arrays.toString(request.getCameras()));
         for (int camera : request.getCameras())
         {
            cameras.add(cameraSettings.get(camera));
         }

         if (request.hasVideoStream())
         {
            videoStreamAddress = new InetSocketAddress(LogUtils.getByAddress(request.getVideoStream()), request.getVideoPort());
            System.out.println("Video stream: " + videoStreamAddress);
         }
         else
         {
            videoStreamAddress = null;
         }

         configuration.close();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Cannot load " + options.getConfigurationFile(), e);
      }

   }

   public boolean changesVariables()
   {
      return false;
   }

   public void receivedHandshake(LogHandshake handshake)
   {
      File handshakeFile = new File(tempDirectory, handshakeFilename);
      try
      {
         FileOutputStream handshakeStream = new FileOutputStream(handshakeFile, false);
         handshakeStream.write(handshake.protoShake);
         handshakeStream.close();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      if (handshake.modelLoaderClass != null)
      {
         logProperties.setModelLoaderClass(handshake.modelLoaderClass);
         logProperties.setModelName(handshake.modelName);
         logProperties.setModelResourceDirectories(handshake.resourceDirectories);
         logProperties.setModelPath(modelFilename);
         logProperties.setModelResourceBundlePath(modelResourceBundle);

         File modelFile = new File(tempDirectory, modelFilename);
         File resourceFile = new File(tempDirectory, modelResourceBundle);
         try
         {
            FileOutputStream modelStream = new FileOutputStream(modelFile, false);
            modelStream.write(handshake.model);
            modelStream.close();
            FileOutputStream resourceStream = new FileOutputStream(resourceFile, false);
            resourceStream.write(handshake.resourceZip);
            resourceStream.close();

         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }

      }

   }

   public void receivedUpdate(long timestamp, ByteBuffer buffer)
   {
      connected = true;

      synchronized (synchronizer)
      {
         if (!clearingLog && dataChannel != null)
         {
            try
            {
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

      try
      {
         logProperties.store();
      }
      catch (IOException e)
      {
         e.printStackTrace();
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
         if (options.isEnableCookieJar())
         {
            System.out.println("Creating cookiejar");
            File cookieJarDirectory = new File(tempDirectory, "cookieJar");
            cookieJarDirectory.mkdir();
            CookieJar cookieJar = new CookieJar();
            cookieJar.setDirectory(cookieJarDirectory.getAbsolutePath());
            cookieJar.setHost(options.getCookieJarHost());
            cookieJar.setUser(options.getCookieJarUser());
            cookieJar.setRemoteDirectory(options.getCookieJarRemoteDirectory());

            PipedCommandExecutor executor = new PipedCommandExecutor(cookieJar);
            try
            {
               executor.execute();
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
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
         yoVariableClient.close();
      }
      else
      {
         System.out.println("Cannot connect to client, closing");
         yoVariableClient.close();
      }
   }

   public boolean populateRegistry()
   {
      return false;
   }

   @Override
   public long getDisplayOneInNPackets()
   {
      return 1;
   }

   @Override
   public void start(LogModelLoader logModelLoader, YoVariableRegistry yoVariableRegistry, List<JointState<? extends Joint>> list,
         YoGraphicsListRegistry yoGraphicsListRegistry, int bufferSize, boolean showOverheadView)
   {
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
         for (VideoSettings camera : cameras)
         {
            try
            {
               videoDataLoggers.add(new BlackmagicVideoDataLogger(tempDirectory, logProperties, camera, options));
            }
            catch (IOException e)
            {
               System.err.println("Cannot start video data logger");
               e.printStackTrace();
            }
         }

         if (videoStreamAddress != null)
         {
            try
            {
               videoDataLoggers.add(new NetworkStreamVideoDataLogger(request, tempDirectory, logProperties, options, videoStreamAddress));
            }
            catch (IOException e)
            {
               System.err.println("Cannot start video stream logger");
               e.printStackTrace();
            }
         }
      }
   }

   @Override
   public void timestampReceived(long timestamp)
   {
      for(int i = 0; i < videoDataLoggers.size(); i++)
      {
         videoDataLoggers.get(i).timestampChanged(timestamp);
      }
   }

   @Override
   public void clearLog()
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
      synchronized (synchronizer)
      {
         clearingLog = false;
      }
   }

}
