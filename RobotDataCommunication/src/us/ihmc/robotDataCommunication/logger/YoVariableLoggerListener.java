package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
   private static final long connectTimeout = 20000;
   private static final long disconnectTimeout = 5000;
   private static final String handshakeFilename = "handshake.proto";
   private static final String dataFilename = "robotData.bin";
   private static final String modelFilename = "model.sdf";
   private static final String modelResourceBundle = "resources.zip";
   private static final String indexFilename = "robotData.dat";

   private final Object synchronizer = new Object();

   private final File directory;
   private final YoVariableLoggerOptions options;
   private FileChannel dataChannel;
   private FileChannel indexChannel;

   private final ByteBuffer indexBuffer = ByteBuffer.allocate(8);
   private ByteBuffer compressedBuffer;
   
   private YoVariableClient yoVariableClient;
   private volatile boolean connected = false;
   private long totalTimeout = 0;

   private final LogPropertiesWriter logProperties;
   private ArrayList<VideoDataLogger> videoDataLoggers = new ArrayList<VideoDataLogger>();

   private final ArrayList<VideoSettings> cameras = new ArrayList<>();
   
   public YoVariableLoggerListener(File directory, AnnounceRequest request, YoVariableLoggerOptions options)
   {
      this.directory = directory;
      this.options = options;
      logProperties = new LogPropertiesWriter(new File(directory, propertyFile));
      logProperties.setHandshakeFile(handshakeFilename);
      logProperties.setVariableDataFile(dataFilename);
      logProperties.setCompressed(true);
      logProperties.setVariablesIndexFile(indexFilename);
      try
      {
         FileInputStream configuration = new FileInputStream(options.getConfigurationFile());
         List<VideoSettings> cameraSettings = VideoSettings.loadCameraSettings(configuration);

         System.out.println("Cameras: " + Arrays.toString(request.getCameras()));
         for (int camera : request.getCameras())
         {
            cameras.add(cameraSettings.get(camera));
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
      File handshakeFile = new File(directory, handshakeFilename);
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

         File modelFile = new File(directory, modelFilename);
         File resourceFile = new File(directory, modelResourceBundle);
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
      totalTimeout = 0;
      

      synchronized (synchronizer)
      {
         if (dataChannel != null)
         {
            try
            {
               buffer.clear();
               compressedBuffer.clear();
               SnappyUtils.compress(buffer, compressedBuffer);
               compressedBuffer.flip();
               
               
               indexBuffer.clear();
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

      for (VideoDataLogger videoDataLogger : videoDataLoggers)
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
         for (VideoDataLogger videoDataLogger : videoDataLoggers)
         {
            videoDataLogger.removeLogFiles();
         }

         File handshakeFile = new File(directory, handshakeFilename);
         if (handshakeFile.exists())
         {
            System.out.println("Deleting handshake file");
            handshakeFile.delete();
         }

         File properties = new File(directory, propertyFile);
         if (properties.exists())
         {
            System.out.println("Deleting properties file");
            properties.delete();
         }

         File model = new File(directory, modelFilename);
         if (model.exists())
         {
            System.out.println("Deleting model file");
            model.delete();
         }

         File resources = new File(directory, modelResourceBundle);
         {
            System.out.println("Deleting resource bundle");
            resources.delete();
         }

         File dataFile = new File(directory, dataFilename);
         if (dataFile.exists())
         {
            System.out.println("Deleting data file");
            dataFile.delete();
         }

         File indexFile = new File(directory, indexFilename);
         if (indexFile.exists())
         {
            System.out.println("Deleting index file");
            indexFile.delete();
         }

         if (directory.exists())
         {
            System.out.println("Deleting log directory");
            directory.delete();
         }

      }
      else if (options.isEnableCookieJar())
      {
         System.out.println("Creating cookiejar");
         File cookieJarDirectory = new File(directory, "cookieJar");
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
   }

   public void setYoVariableClient(YoVariableClient client)
   {
      this.yoVariableClient = client;
   }

   public void receiveTimedOut(long timeoutInMillis)
   {
      totalTimeout += timeoutInMillis;
      if (connected)
      {
         if (totalTimeout > disconnectTimeout)
         {
            System.out.println("Timeout reached: " + totalTimeout + ". Connection lost, closing client.");
            yoVariableClient.close();
         }
      }
      else
      {
         if (totalTimeout > connectTimeout)
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
      File dataFile = new File(directory, dataFilename);
      File indexFile = new File(directory, indexFilename);
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
         try
         {
            for (VideoSettings camera : cameras)
            {
               videoDataLoggers.add(new VideoDataLogger(directory, logProperties, camera, options));
            }
         }
         catch (IOException e)
         {
            System.err.println("Cannot start video data logger");
            e.printStackTrace();
         }
      }
   }

   @Override
   public void timestampReceived(long timestamp)
   {
      for (VideoDataLogger videoDataLogger : videoDataLoggers)
      {
         videoDataLogger.timestampChanged(timestamp);
      }
   }

}
