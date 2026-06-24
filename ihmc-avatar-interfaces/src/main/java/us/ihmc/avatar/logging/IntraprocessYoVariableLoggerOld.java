package us.ihmc.avatar.logging;

import logger_msgs.Handshake;
import logger_msgs.HandshakeFileType;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.idl.serializers.extra.ROS2YAMLSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.handshake.YoVariableHandShakeBuilder;
import us.ihmc.robotDataLogger.jointState.JointHolder;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.logger.LogPropertiesWriter;
import us.ihmc.tools.compression.SnappyUtils;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.BufferOverflowException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import javax.annotation.Nullable;

/**
 * Meant to be deleted once logger is released with the correct IntraprocessYoVariableLogger
 */
@Deprecated
public class IntraprocessYoVariableLoggerOld
{
   private static final String INTRAPROCESS_LOG_POSTFIX = "_IntraprocessLogger";
   public static final String PROPERTY_FILE = "robotData.log";
   public static final String HANDSHAKE_FILENAME = "handshake.yaml";
   public static final String DATA_FILENAME = "robotData.bsz";
   public static final String MODEL_FILENAME = "model.sdf";
   public static final String MODEL_RESOURCE_BUNDLE = "resources.zip";
   public static final String INDEX_FILENAME = "robotData.dat";
   public static final Path DEFAULT_INCOMING_LOGS_DIRECTORY = Paths.get(System.getProperty("user.home")).resolve(".ihmc").resolve("logs");
   public static final Path BACKUP_INCOMING_LOGS_DIRECTORY = Paths.get(System.getProperty("user.home")).resolve(".ihmc").resolve("logs2");

   private final List<RegistrySendBufferBuilder> registrySendBufferBuilders;
   private final double dt;
   private final String logName;
   @Nullable
   private final LogModelProvider logModelProvider;

   private Path logFolder;
   private List<YoVariable> variables;
   private long[] variableValues;
   private List<JointHolder> jointHolders;
   private ByteBuffer dataBuffer;
   private LongBuffer dataBufferAsLong;
   private boolean destroyed;

   /*
    * Compression and serialization
    */
   private ByteBuffer compressedBuffer;
   private ByteBuffer indexBuffer;
   private ConcurrentRingBuffer<ByteBuffer> compressionBufferRing;
   private final Object compressionThreadLock = new Object();
   private RepeatingTaskThread compressionThread;
   private FileChannel dataChannel;
   private FileChannel indexChannel;

   public IntraprocessYoVariableLoggerOld(List<RegistrySendBufferBuilder> registrySendBufferBuilders, double dt, String logName)
   {
      this(registrySendBufferBuilders, dt, logName, null);
   }

   public IntraprocessYoVariableLoggerOld(List<RegistrySendBufferBuilder> registrySendBufferBuilders,
                                          double dt,
                                          String logName,
                                          @Nullable LogModelProvider logModelProvider)
   {
      this.registrySendBufferBuilders = registrySendBufferBuilders;
      this.dt = dt;
      this.logName = logName;
      this.logModelProvider = logModelProvider;

      destroyed = true;
   }

   public boolean create()
   {
      try
      {
         DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmssSSS");
         String fileTimestamp = dateFormat.format(Calendar.getInstance().getTime());
         logFolder = DEFAULT_INCOMING_LOGS_DIRECTORY.resolve(fileTimestamp + logName + INTRAPROCESS_LOG_POSTFIX);
         FileTools.ensureDirectoryExists(logFolder);

         Path tempFile = logFolder.resolve(".temp");
         FileTools.deleteQuietly(tempFile);
         if (!tempFile.toFile().createNewFile())
         {
            // If we failed to create a temp file, try the backup logs dir
            logFolder = BACKUP_INCOMING_LOGS_DIRECTORY;
            FileTools.ensureDirectoryExists(logFolder);

            if (!tempFile.toFile().createNewFile())
            {
               // We failed to use both the default and backup logs dir
               return false;
            }
         }
         FileTools.deleteQuietly(tempFile);

         YoVariableHandShakeBuilder handshakeBuilder = new YoVariableHandShakeBuilder("main", dt);
         handshakeBuilder.setFrames(ReferenceFrame.getWorldFrame());

         for (int i = 0; i < registrySendBufferBuilders.size(); i++)
            handshakeBuilder.addRegistryBuffer(registrySendBufferBuilders.get(i));

         Handshake handshake = handshakeBuilder.getHandShake();

         ROS2YAMLSerializer<Handshake> serializer = new ROS2YAMLSerializer<>(Handshake.class);
         serializer.serialize(createFileInLogFolder(HANDSHAKE_FILENAME), handshake);

         LogPropertiesWriter logProperties = new LogPropertiesWriter(createFileInLogFolder(PROPERTY_FILE));
         logProperties.getVariables().setHandshake(HANDSHAKE_FILENAME);
         logProperties.getVariables().setData(DATA_FILENAME);
         logProperties.getVariables().setCompressed(true);
         logProperties.getVariables().setTimestamped(true);
         logProperties.getVariables().setIndex(INDEX_FILENAME);
         logProperties.getVariables().setHandshakeFileType(HandshakeFileType.IDL_YAML);
         logProperties.setName(logName);
         logProperties.setTimestamp(fileTimestamp);

         if (logModelProvider != null)
         {
            logProperties.getModel().setLoader(logModelProvider.getLoader().getCanonicalName());
            logProperties.getModel().setName(logModelProvider.getModelName());
            for (int i = 0; i < logModelProvider.getTopLevelResourceDirectories().length; i++)
            {
               logProperties.getModel().getResourceDirectoriesList().add(logModelProvider.getTopLevelResourceDirectories()[i]);
            }

            logProperties.getModel().setPath(MODEL_FILENAME);
            logProperties.getModel().setResourceBundle(MODEL_RESOURCE_BUNDLE);

            try (FileOutputStream modelStream = new FileOutputStream(createFileInLogFolder(MODEL_FILENAME), false);
                 FileOutputStream resourceStream = new FileOutputStream(createFileInLogFolder(MODEL_RESOURCE_BUNDLE), false))
            {
               modelStream.write(logModelProvider.getModel());
               modelStream.getFD().sync();
               resourceStream.write(logModelProvider.getResourceZip());
               resourceStream.getFD().sync();
            }
         }

         logProperties.store();

         variables = new ArrayList<>();
         for (int i = 0; i < registrySendBufferBuilders.size(); i++)
         {
            variables.addAll(registrySendBufferBuilders.get(i).getYoRegistry().collectSubtreeVariables());
         }

         int numJointStateVars = 0;
         for (int i = 0; i < handshake.getJoints().size(); i++)
         {
            numJointStateVars += JointState.getNumberOfVariables(handshake.getJoints().get(i).getType());
         }

         int singleTickBufferSize = (1 + variables.size() + numJointStateVars) * Long.BYTES;
         dataBuffer = ByteBuffer.allocate(singleTickBufferSize);
         dataBufferAsLong = dataBuffer.asLongBuffer();
         compressedBuffer = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(singleTickBufferSize));
         jointHolders = handshakeBuilder.getJointHolders();
         variableValues = new long[variables.size()];

         long numYoGraphics = registrySendBufferBuilders.stream()
                                                        .filter(b -> b.getSCS2YoGraphics() != null)
                                                        .mapToLong(b -> b.getSCS2YoGraphics().getChildren().size()).sum();

         LogTools.info("Buffer size: {}", singleTickBufferSize);
         LogTools.info("Number of YoVariables: {}", variables.size());
         LogTools.info("Number of YoGraphics: {}", numYoGraphics);
         LogTools.info("Number of joint states: {}", numJointStateVars);

         /*
          * Compression and serialization
          */
         indexBuffer = ByteBuffer.allocate(2 * Long.BYTES);
         compressionBufferRing = new ConcurrentRingBuffer<>(() -> ByteBuffer.allocate(singleTickBufferSize), 2);
         compressionThread = new RepeatingTaskThread("IntraprocessLoggerCompressionThread", () ->
         {
            synchronized (compressionThreadLock)
            {
               compressionThreadLock.wait();
            }

            if (compressionBufferRing.poll())
            {
               ByteBuffer data;

               while ((data = compressionBufferRing.read()) != null)
               {
                  compressedBuffer.clear();
                  data.position(0);
                  SnappyUtils.compress(data, compressedBuffer);
                  compressedBuffer.flip();

                  long timestamp = data.getLong(0);

                  indexBuffer.clear();
                  indexBuffer.putLong(timestamp);
                  indexBuffer.putLong(dataChannel.position());
                  indexBuffer.flip();

                  try
                  {
                     long ignored0 = indexChannel.write(indexBuffer);
                     long ignored1 = dataChannel.write(compressedBuffer);
                  }
                  catch (IOException e)
                  {
                     LogTools.error("Could not log to: " + logFolder.toAbsolutePath());
                     LogTools.error("Stopping Intraprocess logger...");

                     destroy();

                     compressionThread.blockingKill();
                  }
               }

               compressionBufferRing.flush();
            }
         });
         dataChannel = new FileOutputStream(createFileInLogFolder(DATA_FILENAME), false).getChannel();
         indexChannel = new FileOutputStream(createFileInLogFolder(INDEX_FILENAME), false).getChannel();
         dataChannel.force(true);
         indexChannel.force(true);

         compressionThread.startRepeating();

         destroyed = false;

      }
      catch (Exception e)
      {
         return false;
      }

      return true;
   }

   public boolean isDestroyed()
   {
      return destroyed;
   }

   public synchronized void destroy()
   {
      destroyed = true;
      compressionThread.blockingKill();
      try
      {
         dataChannel.close();
         indexChannel.close();
      }
      catch (IOException e)
      {
         LogTools.error(e);
      }
   }

   private final double[] jointData = new double[13];

   public synchronized void update(long timestamp)
   {
      if (destroyed)
      {
         return;
      }

      dataBuffer.clear();
      dataBufferAsLong.clear();
      dataBufferAsLong.put(timestamp);

      try
      {
         for (int i = 0; i < variables.size(); i++)
            variableValues[i] = variables.get(i).getValueAsLongBits();

         dataBufferAsLong.put(variableValues, 0, variables.size());
      }
      catch (BufferOverflowException e)
      {
         LogTools.error("Increase buffer size! size: {} {}", variables.size(), e.getMessage());
      }

      for (int i = 0; i < jointHolders.size(); i++)
      {
         JointHolder jointHolder = jointHolders.get(i);
         jointHolder.get(jointData, 0);
         for (int j = 0; j < jointHolder.getNumberOfStateVariables(); j++)
            dataBufferAsLong.put(Double.doubleToLongBits(jointData[j]));
      }

      dataBufferAsLong.flip();
      dataBuffer.position(0);
      dataBuffer.limit(dataBufferAsLong.limit() * Long.BYTES);

      ByteBuffer writeBuffer = compressionBufferRing.next();

      if (writeBuffer != null)
      {
         writeBuffer.position(0);
         writeBuffer.put(dataBuffer);

         compressionBufferRing.commit();
      }

      synchronized (compressionThreadLock)
      {
         compressionThreadLock.notify();
      }
   }

   private File createFileInLogFolder(String filename)
   {
      FileTools.ensureDirectoryExists(logFolder, DefaultExceptionHandler.RUNTIME_EXCEPTION);
      return logFolder.resolve(filename).toFile();
   }
}
