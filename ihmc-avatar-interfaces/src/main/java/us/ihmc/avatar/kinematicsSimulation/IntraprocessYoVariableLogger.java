package us.ihmc.avatar.kinematicsSimulation;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.idl.serializers.extra.YAMLSerializer;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakeFileType;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandShakeBuilder;
import us.ihmc.robotDataLogger.logger.LogPropertiesWriter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;

public class IntraprocessYoVariableLogger
{
   public static final String propertyFile = "robotData.log";
   public static final String handshakeFilename = "handshake.yaml";
   public static final String dataFilename = "robotData.bsz";
   public static final String modelFilename = "model.sdf";
   public static final String modelResourceBundle = "resources.zip";
   public static final String indexFilename = "robotData.dat";
   public static final String summaryFilename = "summary.csv";
   private final String timestamp;
   private String logFolder;

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
      logFolder = timestamp + "_Log";

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
         serializer.serialize(createFileInLogFolder(handshakeFilename), handshake);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      LogPropertiesWriter logProperties = new LogPropertiesWriter(createFileInLogFolder("robotData.log"));
      logProperties.getVariables().setHandshake(handshakeFilename);
      logProperties.getVariables().setData(dataFilename);
      logProperties.getVariables().setCompressed(true);
      logProperties.getVariables().setTimestamped(true);
      logProperties.getVariables().setIndex(indexFilename);
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
      logProperties.getModel().setPath(modelFilename);
      logProperties.getModel().setResourceBundle(modelResourceBundle);

      File modelFile = createFileInLogFolder(modelFilename);
      File resourceFile = createFileInLogFolder(modelResourceBundle);
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
   }

   private File createFileInLogFolder(String filename)
   {
      FileTools.ensureDirectoryExists(Paths.get(logFolder), DefaultExceptionHandler.RUNTIME_EXCEPTION);
      return new File(logFolder + "/" + filename);
   }
}
