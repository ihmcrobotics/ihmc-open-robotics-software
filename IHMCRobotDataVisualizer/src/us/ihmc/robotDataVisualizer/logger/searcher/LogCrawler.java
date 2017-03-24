package us.ihmc.robotDataVisualizer.logger.searcher;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import us.ihmc.commons.Conversions;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFModelLoader;
import us.ihmc.robotDataLogger.handshake.ProtoBufferYoVariableHandshakeParser;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;
import us.ihmc.robotDataVisualizer.logger.converters.LogFormatUpdater;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class LogCrawler implements Runnable
{
   private SpecificLogVariableUpdater robot;
   private String logFileName;
   private LogCrawlerListenerInterface playbackListener;
   private double dt;
   private File logFile;

   public LogCrawler(File logFile, LogCrawlerListenerInterface playbackListener) throws IOException
   {
      this.logFile = logFile;
      logFileName = logFile.getName();
      this.playbackListener = playbackListener;
   }

   private void readLogFile(File selectedFile) throws IOException
   {
      LogPropertiesReader logProperties = new LogPropertiesReader(new File(selectedFile, YoVariableLoggerListener.propertyFile));
      LogFormatUpdater.updateLogs(selectedFile, logProperties);
      logProperties.getCameras().clear();

      File handshake = new File(selectedFile, logProperties.getVariables().getHandshakeAsString());
      if (!handshake.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getVariables().getHandshakeAsString());
      }

      DataInputStream handshakeStream = new DataInputStream(new FileInputStream(handshake));
      byte[] handshakeData = new byte[(int) handshake.length()];
      handshakeStream.readFully(handshakeData);
      handshakeStream.close();

      YoVariableHandshakeParser parser = YoVariableHandshakeParser.create(logProperties.getVariables().getHandshakeFileType(), "logged");
      parser.parseFrom(handshakeData);
      YoVariable<?>[] yoVariablesToUpdate = playbackListener.getYovariablesToUpdate(parser.getRootRegistry());

      GeneralizedSDFRobotModel generalizedSDFRobotModel;
      if (!logProperties.getModel().getLoaderAsString().isEmpty())
      {
         SDFModelLoader loader = new SDFModelLoader();
         String modelName = logProperties.getModel().getNameAsString();
         String[] resourceDirectories = logProperties.getModel().getResourceDirectoriesList().toStringArray();

         File model = new File(selectedFile, logProperties.getModel().getPathAsString());
         DataInputStream modelStream = new DataInputStream(new FileInputStream(model));
         byte[] modelData = new byte[(int) model.length()];
         modelStream.readFully(modelData);
         modelStream.close();

         File resourceBundle = new File(selectedFile, logProperties.getModel().getResourceBundleAsString());
         DataInputStream resourceStream = new DataInputStream(new FileInputStream(resourceBundle));
         byte[] resourceData = new byte[(int) resourceBundle.length()];
         resourceStream.readFully(resourceData);
         resourceStream.close();

         loader.load(modelName, modelData, resourceDirectories, resourceData, null);
         generalizedSDFRobotModel = loader.createJaxbSDFLoader().getGeneralizedSDFRobotModel(modelName);
      }
      else
      {
         throw new RuntimeException("No model available for log");
      }

      boolean useCollisionMeshes = false;

      RobotDescriptionFromSDFLoader loader = new RobotDescriptionFromSDFLoader();
      RobotDescription robotDescription = loader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, null, useCollisionMeshes);

      robot = new SpecificLogVariableUpdater(selectedFile, robotDescription, parser.getJointStates(), parser.getYoVariablesList(), logProperties,
            yoVariablesToUpdate);
      dt = parser.getDt();
   }

   public void run()
   {
      System.out.println("loading log from folder:" + logFile);
      long startTime = System.currentTimeMillis();

      try
      {
         readLogFile(logFile);

         while (!robot.readAndProcessALogLineReturnTrueIfDone(dt))
         {
            playbackListener.update(this, robot.getTime());
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      } 
      finally
      {
         robot.close();
         long endTime = System.currentTimeMillis();
         System.out.println("Finished searching " + logFileName + ", took " + Conversions.millisecondsToMinutes(endTime - startTime) + " minutes");
         playbackListener.onFinish();
      }
   }

   public String getLogFileName()
   {
      return logFileName;
   }
}
