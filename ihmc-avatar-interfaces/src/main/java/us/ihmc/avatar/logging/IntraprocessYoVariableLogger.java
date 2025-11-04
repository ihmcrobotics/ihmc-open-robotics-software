package us.ihmc.avatar.logging;

import com.google.common.collect.Lists;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.idl.serializers.extra.YAMLSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.*;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.handshake.YoVariableHandShakeBuilder;
import us.ihmc.robotDataLogger.jointState.JointHolder;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.logger.LogPropertiesWriter;
import us.ihmc.tools.compression.SnappyUtils;
import us.ihmc.yoVariables.registry.YoRegistry;
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
import java.util.*;

public class IntraprocessYoVariableLogger
{
   private static final String INTRAPROCESS_LOG_POSTFIX = "_IntraprocessLogger";
   public static final String PROPERTY_FILE = "robotData.log";
   public static final String HANDSHAKE_FILENAME = "handshake.yaml";
   public static final String DATA_FILENAME = "robotData.bsz";
   public static final String MODEL_FILENAME = "model.sdf";
   public static final String MODEL_RESOURCE_BUNDLE = "resources.zip";
   public static final String INDEX_FILENAME = "robotData.dat";
   public static final Path DEFAULT_INCOMING_LOGS_DIRECTORY;
   static
   {
      Path incomingLogsDirectory = Paths.get(System.getProperty("user.home")).resolve(".ihmc");
      if (ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         // TODO GITHUB WORKFLOWS
//         incomingLogsDirectory = incomingLogsDirectory.resolve("bamboo-logs")
//                                                      .resolve(System.getenv("bamboo_planKey"))
//                                                      .resolve(System.getenv("bamboo_buildResultKey"));
      }
      else
      {
         incomingLogsDirectory = incomingLogsDirectory.resolve("logs");
      }
      DEFAULT_INCOMING_LOGS_DIRECTORY = incomingLogsDirectory;
   }

   private final String logName;
   private final LogModelProvider logModelProvider;
   private final List<RegistrySendBufferBuilder> registrySendBufferBuilders;
   private final int maxTicksToRecord;
   private final double dt;
   private final Path incomingLogsFolder;

   private ByteBuffer compressedBuffer;
   private Path logFolder;
   private final ByteBuffer indexBuffer = ByteBuffer.allocate(16);
   private final ArrayList<YoVariable> variables = new ArrayList<>();
   private List<JointHolder> jointHolders;
   private ByteBuffer dataBuffer;
   private LongBuffer dataBufferAsLong;
   private FileChannel dataChannel;
   private FileChannel indexChannel;
   private volatile boolean shutdown = false;
   private long[] variableValues;

   // Added for async compression
   public static final int TASK_POOL_SIZE = 10;
   private final RepeatingTaskThread compressionTaskThread = new RepeatingTaskThread("IntraprocessLoggerCompressionThread", this::compressLatestData);
   public final RecyclingArrayList<CompressionTask> compressionQueue = new RecyclingArrayList<>(TASK_POOL_SIZE, CompressionTask.class);
   private int nextTaskIndex = 0;
   private int nextCompressionIndex = 0;

   private final double[] jointData = new double[13];

   public IntraprocessYoVariableLogger(String logName,
                                       LogModelProvider logModelProvider,
                                       YoRegistry registry,
                                       RigidBodyBasics rootBody,
                                       YoGraphicsListRegistry yoGraphicsListRegistry,
                                       int maxTicksToRecord,
                                       double dt)
   {
      this(logName,
           logModelProvider,
           Lists.newArrayList(new RegistrySendBufferBuilder(registry, rootBody, yoGraphicsListRegistry)),
           maxTicksToRecord,
           dt);
   }

   public IntraprocessYoVariableLogger(String logName,
                                       YoRegistry registry,
                                       int maxTicksToRecord,
                                       double dt)
   {
      this(logName,
           null,
           Lists.newArrayList(new RegistrySendBufferBuilder(registry)),
           maxTicksToRecord,
           dt);
   }

   public IntraprocessYoVariableLogger(String logName,
                                       LogModelProvider logModelProvider,
                                       List<RegistrySendBufferBuilder> registrySendBufferBuilders,
                                       int maxTicksToRecord,
                                       double dt)
   {
      this.logName = logName;
      this.logModelProvider = logModelProvider;
      this.registrySendBufferBuilders = registrySendBufferBuilders;
      this.maxTicksToRecord = maxTicksToRecord;
      this.dt = dt;
      this.incomingLogsFolder = DEFAULT_INCOMING_LOGS_DIRECTORY;
   }

   public void start()
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmssSSS");
      Calendar calendar = Calendar.getInstance();
      String timestamp = dateFormat.format(calendar.getTime());
      logFolder = incomingLogsFolder.resolve(timestamp + logName + INTRAPROCESS_LOG_POSTFIX);

      YoVariableHandShakeBuilder handshakeBuilder = new YoVariableHandShakeBuilder("main", dt);  // might not want this
      handshakeBuilder.setFrames(ReferenceFrame.getWorldFrame());
      for (RegistrySendBufferBuilder registrySendBufferBuilder : registrySendBufferBuilders)
      {
         handshakeBuilder.addRegistryBuffer(registrySendBufferBuilder);
      }
      Handshake handshake = handshakeBuilder.getHandShake();

      try
      {
         YAMLSerializer<Handshake> serializer = new YAMLSerializer<>(new HandshakePubSubType());
         serializer.serialize(createFileInLogFolder(HANDSHAKE_FILENAME), handshake);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      LogPropertiesWriter logProperties = new LogPropertiesWriter(createFileInLogFolder(PROPERTY_FILE));
      logProperties.getVariables().setHandshake(HANDSHAKE_FILENAME);
      logProperties.getVariables().setData(DATA_FILENAME);
      logProperties.getVariables().setCompressed(true);
      logProperties.getVariables().setTimestamped(true);
      logProperties.getVariables().setIndex(INDEX_FILENAME);
      logProperties.getVariables().setHandshakeFileType(HandshakeFileType.IDL_YAML);

      logProperties.setName(logName);
      logProperties.setTimestamp(timestamp);

      // create resource zip
      if (logModelProvider != null)
      {
         logProperties.getModel().setLoader(logModelProvider.getLoader().getCanonicalName());
         logProperties.getModel().setName(logModelProvider.getModelName());
         for (String resourceDirectory : logModelProvider.getTopLevelResourceDirectories())
         {
            logProperties.getModel().getResourceDirectoriesList().add(resourceDirectory);
         }
         logProperties.getModel().setPath(MODEL_FILENAME);
         logProperties.getModel().setResourceBundle(MODEL_RESOURCE_BUNDLE);

         File modelFile = createFileInLogFolder(MODEL_FILENAME);
         File resourceFile = createFileInLogFolder(MODEL_RESOURCE_BUNDLE);
         try
         {
            FileOutputStream modelStream = new FileOutputStream(modelFile, false);
            modelStream.write(logModelProvider.getModel());
            modelStream.getFD().sync();
            modelStream.close();
            FileOutputStream resourceStream = new FileOutputStream(resourceFile, false);
            resourceStream.write(logModelProvider.getResourceZip());
            resourceStream.getFD().sync();
            resourceStream.close();
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
      }

      try
      {
         logProperties.store();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      int numberOfJointStates = 0;
      for (int i = 0; i < handshake.getJoints().size(); i++)
      {
         JointDefinition joint = handshake.getJoints().get(i);
         numberOfJointStates += JointState.getNumberOfVariables(joint.getType());
      }

      int stateVariables = 1 + maxTicksToRecord + numberOfJointStates; // for some reason yovariable registry doesn't have all the variables yet
      int bufferSize = stateVariables * 8;

      for (int i = 0; i < TASK_POOL_SIZE; i++)
      {
         compressionQueue.add();
      }

      compressionTaskThread.start();

      dataBuffer = ByteBuffer.allocate(bufferSize);
      dataBufferAsLong = dataBuffer.asLongBuffer();
      compressedBuffer = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(bufferSize));
      for (RegistrySendBufferBuilder registrySendBufferBuilder : registrySendBufferBuilders)
      {
         variables.addAll(registrySendBufferBuilder.getYoRegistry().collectSubtreeVariables());
      }
      jointHolders = handshakeBuilder.getJointHolders();

      // Setup data holder to not create memory inside the update loop
      variableValues = new long[variables.size()];

      long numberOfYoGraphics = 0;
      for (RegistrySendBufferBuilder registrySendBufferBuilder : registrySendBufferBuilders)
      {
         if (registrySendBufferBuilder.getSCS1YoGraphics() != null)
         {
            for (YoGraphicsList yoGraphicsList : registrySendBufferBuilder.getSCS1YoGraphics().getYoGraphicsLists())
            {
               numberOfYoGraphics += yoGraphicsList.getYoGraphics().size();
            }
         }
      }
      LogTools.info("Buffer size: {}", bufferSize);
      LogTools.info("Number of YoVariables: {}", variables.size());
      LogTools.info("Number of YoGraphics: {}", numberOfYoGraphics);
      LogTools.info("Number of joint states: {}", numberOfJointStates);

      try
      {
         dataChannel = new FileOutputStream(createFileInLogFolder(DATA_FILENAME), false).getChannel();
         indexChannel = new FileOutputStream(createFileInLogFolder(INDEX_FILENAME), false).getChannel();
         dataChannel.force(true);
         indexChannel.force(true);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      Runtime.getRuntime().addShutdownHook(new Thread(this::shutdown, "Shutdown Hook"));
   }

   public synchronized void update(long timestamp)
   {
      if (shutdown)
      {
         LogTools.error("Logger has already shutdown!");
         return;
      }

      dataBuffer.clear();
      dataBufferAsLong.clear();
      dataBufferAsLong.put(timestamp);

      // Array to hold data for YoVariables to put in buffer
      long[] values = variableValues;
      int size = variables.size();

      try
      {
         for (int i = 0; i < size; i++)
            values[i] = variables.get(i).getValueAsLongBits();

         dataBufferAsLong.put(values, 0, size);
      }
      catch (BufferOverflowException e)
      {
         LogTools.error("Increase buffer size!  size: {}  {}", variables.size(), e.getMessage());
      }

      for (int j = 0; j < jointHolders.size(); j++)
      {
         JointHolder jointHolder = jointHolders.get(j);
         jointHolder.get(jointData, 0);
         int n = jointHolder.getNumberOfStateVariables();
         for (int i = 0; i < n; i++)
            dataBufferAsLong.put(Double.doubleToLongBits(jointData[i]));
      }

      dataBufferAsLong.flip();
      dataBuffer.position(0);
      dataBuffer.limit(dataBufferAsLong.limit() * 8);

      // To avoid creating garbage, we have a pool of tasks which we cycle through, acting as a circular array
      compressionQueue.get(nextTaskIndex).set(timestamp, dataBuffer);
      nextTaskIndex = (nextTaskIndex + 1) % TASK_POOL_SIZE;
      compressionTaskThread.addScheduled(1);
   }

   public void compressLatestData()
   {
      try
      {
         CompressionTask task = compressionQueue.get(nextCompressionIndex);
         nextCompressionIndex = (nextCompressionIndex + 1) % TASK_POOL_SIZE;

         ByteBuffer input = task.data;
         compressedBuffer.clear();
         SnappyUtils.compress(input, compressedBuffer);
         compressedBuffer.flip();

         synchronized (this)
         {
            indexBuffer.clear();
            indexBuffer.putLong(task.timestamp);
            indexBuffer.putLong(dataChannel.position());
            indexBuffer.flip();

            indexChannel.write(indexBuffer);
            dataChannel.write(compressedBuffer);
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void shutdown()
   {
      shutdown = true;
      compressionTaskThread.blockingKill();
      synchronized (this)
      {
         try
         {
            LogTools.info("Closing data channel...");
            dataChannel.close();
            LogTools.info("Data channel closed.");
            LogTools.info("Closing index channel...");
            indexChannel.close();
            LogTools.info("Index channel closed.");
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   private File createFileInLogFolder(String filename)
   {
      FileTools.ensureDirectoryExists(logFolder, DefaultExceptionHandler.RUNTIME_EXCEPTION);
      return logFolder.resolve(filename).toFile();
   }

   public static class CompressionTask
   {
      private long timestamp;
      private ByteBuffer data;

      public CompressionTask()
      {

      }

      void set(long timestamp, ByteBuffer data)
      {
         this.timestamp = timestamp;
         this.data = data;
      }
   }
}
