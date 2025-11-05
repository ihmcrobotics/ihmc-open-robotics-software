package us.ihmc.avatar.logging;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.idl.serializers.extra.YAMLSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakeFileType;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.JointDefinition;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.handshake.YoVariableHandShakeBuilder;
import us.ihmc.robotDataLogger.jointState.JointHolder;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.logger.LogPropertiesWriter;
import us.ihmc.tools.compression.SnappyUtils;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.annotation.Nullable;
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
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

public class IntraprocessYoVariableLoggerV2
{
   private static final String INTRAPROCESS_LOG_POSTFIX = "_IntraprocessLogger";
   public static final String PROPERTY_FILE = "robotData.log";
   public static final String HANDSHAKE_FILENAME = "handshake.yaml";
   public static final String DATA_FILENAME = "robotData.bsz";
   public static final String MODEL_FILENAME = "model.sdf";
   public static final String MODEL_RESOURCE_BUNDLE = "resources.zip";
   public static final String INDEX_FILENAME = "robotData.dat";
   public static final String SUMMARY_FILENAME = "summary.csv";
   public static final Path DEFAULT_INCOMING_LOGS_DIRECTORY = Paths.get(System.getProperty("user.home")).resolve(".ihmc").resolve("logs");

   private final List<RegistrySendBufferBuilder> registrySendBufferBuilders;
   private final double dt;
   private final String logName;
   @Nullable
   private final LogModelProvider logModelProvider;

   private Path logFolder;
   private ByteBuffer compressedBuffer;
   private ByteBuffer indexBuffer;
   private List<YoVariable> variables;
   private long[] variableValues;
   private List<JointHolder> jointHolders;
   private ByteBuffer dataBuffer;
   private LongBuffer dataBufferAsLong;
   private boolean destroyed;

   /*
    * Compression and serialization
    *
    * The ByteBuffers from the pool can either be in the "ready" queue or the "used" queue
    */
   private ByteBuffer[] compressionBufferPool;
   private BlockingQueue<ByteBuffer> compressionBuffersReady;
   private BlockingQueue<ByteBuffer> compressionBuffersUsed;
   private RepeatingTaskThread compressionThread;
   private FileChannel dataChannel;
   private FileChannel indexChannel;

   public IntraprocessYoVariableLoggerV2(List<RegistrySendBufferBuilder> registrySendBufferBuilders, double dt, String logName)
   {
      this(registrySendBufferBuilders, dt, logName, null);
   }

   public IntraprocessYoVariableLoggerV2(List<RegistrySendBufferBuilder> registrySendBufferBuilders,
                                         double dt,
                                         String logName,
                                         @Nullable LogModelProvider logModelProvider)
   {
      this.registrySendBufferBuilders = registrySendBufferBuilders;
      this.dt = dt;
      this.logName = logName;
      this.logModelProvider = logModelProvider;
   }

   public void create() throws IOException
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmssSSS");
      String timestamp = dateFormat.format(Calendar.getInstance().getTime());
      logFolder = DEFAULT_INCOMING_LOGS_DIRECTORY.resolve(timestamp + logName + INTRAPROCESS_LOG_POSTFIX);

      YoVariableHandShakeBuilder handshakeBuilder = new YoVariableHandShakeBuilder("main", dt);
      handshakeBuilder.setFrames(ReferenceFrame.getWorldFrame());

      for (RegistrySendBufferBuilder builder : registrySendBufferBuilders)
         handshakeBuilder.addRegistryBuffer(builder);

      Handshake handshake = handshakeBuilder.getHandShake();

      YAMLSerializer<Handshake> serializer = new YAMLSerializer<>(new HandshakePubSubType());
      serializer.serialize(createFileInLogFolder(HANDSHAKE_FILENAME), handshake);

      LogPropertiesWriter logProperties = new LogPropertiesWriter(createFileInLogFolder(PROPERTY_FILE));
      logProperties.getVariables().setHandshake(HANDSHAKE_FILENAME);
      logProperties.getVariables().setData(DATA_FILENAME);
      logProperties.getVariables().setCompressed(true);
      logProperties.getVariables().setTimestamped(true);
      logProperties.getVariables().setIndex(INDEX_FILENAME);
      logProperties.getVariables().setHandshakeFileType(HandshakeFileType.IDL_YAML);
      logProperties.setName(logName);
      logProperties.setTimestamp(timestamp);

      if (logModelProvider != null)
      {
         logProperties.getModel().setLoader(logModelProvider.getLoader().getCanonicalName());
         logProperties.getModel().setName(logModelProvider.getModelName());
         for (String resourceDirectory : logModelProvider.getTopLevelResourceDirectories())
            logProperties.getModel().getResourceDirectoriesList().add(resourceDirectory);
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
      for (RegistrySendBufferBuilder builder : registrySendBufferBuilders)
         variables.addAll(builder.getYoRegistry().collectSubtreeVariables());

      int numJointStateVars = 0;
      for (JointDefinition joint : handshake.getJoints())
         numJointStateVars += JointState.getNumberOfVariables(joint.getType());

      int singleTickBufferSize = (1 + variables.size() + numJointStateVars) * Long.BYTES;
      dataBuffer = ByteBuffer.allocate(singleTickBufferSize);
      dataBufferAsLong = dataBuffer.asLongBuffer();
      compressedBuffer = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(singleTickBufferSize));
      indexBuffer = ByteBuffer.allocate(16);
      jointHolders = handshakeBuilder.getJointHolders();
      variableValues = new long[variables.size()];

      long numYoGraphics = registrySendBufferBuilders.stream()
                                                     .filter(b -> b.getSCS1YoGraphics() != null)
                                                     .flatMap(b -> b.getSCS1YoGraphics().getYoGraphicsLists().stream())
                                                     .mapToLong(list -> list.getYoGraphics().size())
                                                     .sum();

      LogTools.info("Buffer size: {}", singleTickBufferSize);
      LogTools.info("Number of YoVariables: {}", variables.size());
      LogTools.info("Number of YoGraphics: {}", numYoGraphics);
      LogTools.info("Number of joint states: {}", numJointStateVars);

      /*
       * Compression and serialization
       */
      int poolSize = 10;
      compressionBufferPool = new ByteBuffer[poolSize];
      for (int i = 0; i < compressionBufferPool.length; i++)
         compressionBufferPool[i] = ByteBuffer.allocate(singleTickBufferSize);
      compressionBuffersReady = new ArrayBlockingQueue<>(poolSize, true);
      compressionBuffersUsed = new ArrayBlockingQueue<>(poolSize, true);
      compressionThread = new RepeatingTaskThread("IntraprocessLoggerCompressionThread", () ->
      {
         ByteBuffer singleTickData = compressionBuffersUsed.take();

         

      });
      dataChannel = new FileOutputStream(createFileInLogFolder(DATA_FILENAME), false).getChannel();
      indexChannel = new FileOutputStream(createFileInLogFolder(INDEX_FILENAME), false).getChannel();
      dataChannel.force(true);
      indexChannel.force(true);
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
         LogTools.error("Logger has already been destroyed.");
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

      try
      {
         compressedBuffer.clear();
         SnappyUtils.compress(dataBuffer, compressedBuffer);
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
         LogTools.error(e);
      }
   }

   private File createFileInLogFolder(String filename)
   {
      FileTools.ensureDirectoryExists(logFolder, DefaultExceptionHandler.RUNTIME_EXCEPTION);
      return logFolder.resolve(filename).toFile();
   }
}
