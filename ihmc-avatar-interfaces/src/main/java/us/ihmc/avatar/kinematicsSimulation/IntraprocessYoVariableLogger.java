package us.ihmc.avatar.kinematicsSimulation;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.idl.serializers.extra.YAMLSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.*;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandShakeBuilder;
import us.ihmc.robotDataLogger.jointState.JointHolder;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.logger.LogPropertiesWriter;
import us.ihmc.tools.compression.SnappyUtils;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
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
import java.util.Calendar;
import java.util.List;

public class IntraprocessYoVariableLogger
{
   public static final String PROPERTY_FILE = "robotData.log";
   public static final String HANDSHAKE_FILENAME = "handshake.yaml";
   public static final String DATA_FILENAME = "robotData.bsz";
   public static final String MODEL_FILENAME = "model.sdf";
   public static final String MODEL_RESOURCE_BUNDLE = "resources.zip";
   public static final String INDEX_FILENAME = "robotData.dat";
   public static final String SUMMARY_FILENAME = "summary.csv";
   private final String timestamp;
   private final Path ihmcFolder = Paths.get(System.getProperty("user.home")).resolve(".ihmc");
   private Path logFolder;
   private ByteBuffer compressedBuffer;
   private ByteBuffer indexBuffer = ByteBuffer.allocate(16);
   private List<YoVariable<?>> variables;
   private List<JointHolder> jointHolders;
   private ByteBuffer dataBuffer;
   private LongBuffer dataBufferAsLong;
   private FileChannel dataChannel;
   private FileChannel indexChannel;

   private static final int CHANGED_BUFFER_CAPACITY = 128;
   private ConcurrentRingBuffer<VariableChangedMessage> variableChanged = new ConcurrentRingBuffer<>(new VariableChangedMessage.Builder(),
                                                                                                     CHANGED_BUFFER_CAPACITY);

   public IntraprocessYoVariableLogger(String logName,
                                       LogModelProvider logModelProvider,
                                       YoVariableRegistry registry,
                                       RigidBodyBasics rootBody,
                                       YoGraphicsListRegistry yoGraphicsListRegistry,
                                       double dt)
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      Calendar calendar = Calendar.getInstance();
      timestamp = dateFormat.format(calendar.getTime());
      logFolder = ihmcFolder.resolve(timestamp + "_Log");

      RegistrySendBufferBuilder registrySendBufferBuilder = new RegistrySendBufferBuilder(registry, rootBody, yoGraphicsListRegistry);

      YoVariableHandShakeBuilder handshakeBuilder = new YoVariableHandShakeBuilder("main", dt);  // might not want this
      handshakeBuilder.setFrames(ReferenceFrame.getWorldFrame());
      handshakeBuilder.addRegistryBuffer(registrySendBufferBuilder);
      Handshake handshake = handshakeBuilder.getHandShake();

      LogHandshake logHandshake = new LogHandshake();

      ObjectMapper objectMapper = new ObjectMapper(new YAMLFactory());

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

      logProperties.getModel().setLoader(logModelProvider.getLoader().getCanonicalName());
      logProperties.getModel().setName(logModelProvider.getModelName());
      for (String resourceDirectory : logModelProvider.getResourceDirectories())
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

      int stateVariables = 1 + 9876 + numberOfJointStates; // for some reason yovariable registry doesn't have all the variables yet
      int bufferSize = stateVariables * 8;
      LogTools.info("Buffer size: {}", bufferSize);
      LogTools.info("Number of YoVariables: {}", registry.getNumberOfYoVariables());
      LogTools.info("Number of joint states: {}", numberOfJointStates);

      compressedBuffer = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(bufferSize));
      dataBuffer = ByteBuffer.allocate(bufferSize);
      dataBufferAsLong = dataBuffer.asLongBuffer();
      variables = registry.getAllVariables();
      jointHolders = handshakeBuilder.getJointHolders();

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

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         try
         {
            dataChannel.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }));
   }

   public void update(long timestamp)
   {
      dataBuffer.clear();
      dataBufferAsLong.clear();

      dataBufferAsLong.put(timestamp);
      for (int i = 0; i < variables.size(); i++)
      {
         try
         {
            dataBufferAsLong.put(variables.get(i).getValueAsLongBits());
         }
         catch (BufferOverflowException e)
         {
            LogTools.error("Increase buffer size! yoVar # {}:  size: {}  {}", i, variables.size(), e.getMessage());
         }
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
      dataBuffer.clear();

      try
      {
         dataBuffer.clear();
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

   private File createFileInLogFolder(String filename)
   {
      FileTools.ensureDirectoryExists(logFolder, DefaultExceptionHandler.RUNTIME_EXCEPTION);
      return logFolder.resolve(filename).toFile();
   }
}
