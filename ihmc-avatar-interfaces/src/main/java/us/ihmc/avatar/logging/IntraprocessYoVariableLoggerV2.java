package us.ihmc.avatar.logging;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
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

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.BufferOverflowException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

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
   private final LogModelProvider logModelProvider;

   /*
    * Data created at start() time
    */
   private Path logFolder;
   private ByteBuffer compressedBuffer;
   private ByteBuffer indexBuffer;
   private List<YoVariable> variables;
   private List<JointHolder> jointHolders;
   private ByteBuffer dataBuffer;
   private LongBuffer dataBufferAsLong;
   private FileChannel dataChannel;
   private FileChannel indexChannel;
   private boolean shutdown;

   public IntraprocessYoVariableLoggerV2(List<RegistrySendBufferBuilder> registrySendBufferBuilders, double dt, String logName)
   {
      this(registrySendBufferBuilders, dt, logName, null);
   }

   public IntraprocessYoVariableLoggerV2(List<RegistrySendBufferBuilder> registrySendBufferBuilders,
                                         double dt,
                                         String logName,
                                         LogModelProvider logModelProvider)
   {
      this.registrySendBufferBuilders = registrySendBufferBuilders;
      this.dt = dt;
      this.logName = logName;
      this.logModelProvider = logModelProvider;
   }

   public void create()
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmssSSS");
      Calendar calendar = Calendar.getInstance();
      String timestamp = dateFormat.format(calendar.getTime());
      logFolder = DEFAULT_INCOMING_LOGS_DIRECTORY.resolve(timestamp + logName + INTRAPROCESS_LOG_POSTFIX);
      deleteOldLogs(DEFAULT_INCOMING_LOGS_DIRECTORY, 10);

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
         LogTools.error(e);
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

      // Create resource zip
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
         LogTools.error(e);
      }

      int numberOfJointStateVariables = 0;
      for (int i = 0; i < handshake.getJoints().size(); i++)
      {
         JointDefinition joint = handshake.getJoints().get(i);
         numberOfJointStateVariables += JointState.getNumberOfVariables(joint.getType());
      }
      int numberOfVariables = 0;
      for (RegistrySendBufferBuilder registrySendBufferBuilder : registrySendBufferBuilders)
      {
         numberOfVariables += registrySendBufferBuilder.getNumberOfVariables();
      }
      int singleTickBufferSize = (1 + numberOfVariables + numberOfJointStateVariables) * Long.BYTES;

      dataBuffer = ByteBuffer.allocate(singleTickBufferSize);
      dataBufferAsLong = dataBuffer.asLongBuffer();
      compressedBuffer = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(singleTickBufferSize));
      indexBuffer = ByteBuffer.allocate(16);

      variables = new ArrayList<>();
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
      LogTools.info("Buffer size: {}", singleTickBufferSize);
      LogTools.info("Number of YoVariables: {}", variables.size());
      LogTools.info("Number of YoGraphics: {}", numberOfYoGraphics);
      LogTools.info("Number of joint states: {}", numberOfJointStateVariables);

      try
      {
         dataChannel = new FileOutputStream(createFileInLogFolder(DATA_FILENAME), false).getChannel();
         indexChannel = new FileOutputStream(createFileInLogFolder(INDEX_FILENAME), false).getChannel();
         dataChannel.force(true);
         indexChannel.force(true);
      }
      catch (Exception e)
      {
         LogTools.error(e);
      }
   }

   public synchronized void destroy()
   {
      shutdown = true;

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

   private long[] variableValues;

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

      long[] values = variableValues;
      int size = variables.size();

      try
      {
         for (int i = 0; i < size; i++)
         {
            values[i] = variables.get(i).getValueAsLongBits();
         }

         dataBufferAsLong.put(values, 0, size);
      }
      catch (BufferOverflowException e)
      {
         LogTools.error("Increase buffer size!  size: {}  {}", variables.size(), e.getMessage());
      }

      double[] jointData = new double[13];
      for (JointHolder jointHolder : jointHolders)
      {
         jointHolder.get(jointData, 0);

         for (int i = 0; i < jointHolder.getNumberOfStateVariables(); i++)
         {
            dataBufferAsLong.put(Double.doubleToLongBits(jointData[i]));
         }
      }

      dataBufferAsLong.flip();
      dataBuffer.position(0);
      dataBuffer.limit(dataBufferAsLong.limit() * 8);

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
         e.printStackTrace();
      }
   }

   public void deleteOldLogs(Path incomingLogsFolder, int numberOflogsToKeep)
   {
      SortedSet<Path> sortedSet = new TreeSet<>(Comparator.comparing(path1 -> path1.getFileName().toString()));
      PathTools.walkFlat(incomingLogsFolder, (path, type) ->
      {
         if (type == BasicPathVisitor.PathType.DIRECTORY && path.getFileName().toString().endsWith(INTRAPROCESS_LOG_POSTFIX))
            sortedSet.add(path);
         return FileVisitResult.CONTINUE;
      });

      while (sortedSet.size() > numberOflogsToKeep)
      {
         Path earliestLogDirectory = sortedSet.first();
         LogTools.warn("Deleting old log {}", earliestLogDirectory);
         FileTools.deleteQuietly(earliestLogDirectory);
         sortedSet.remove(earliestLogDirectory);
      }
   }

   private File createFileInLogFolder(String filename)
   {
      FileTools.ensureDirectoryExists(logFolder, DefaultExceptionHandler.RUNTIME_EXCEPTION);
      return logFolder.resolve(filename).toFile();
   }
}
