package us.ihmc.valkyrie;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.EnumMap;
import java.util.Properties;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessor;
import us.ihmc.communication.producers.VideoControlSettings;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieAdaptiveSwingParameters;
import us.ihmc.valkyrie.externalForceEstimation.ValkyrieExternalForceEstimationModule;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlanner;
import us.ihmc.valkyrie.sensors.ValkyrieSensorSuiteManager;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

public class ValkyrieNetworkProcessor
{
   private enum NetworkProcessorVersion
   {
      IHMC, JSC;

      public static NetworkProcessorVersion fromEnvironment()
      {
         String valueFromEnvironment = System.getenv("IHMC_VALKYRIE_CONFIGURATION");

         if (valueFromEnvironment != null && (valueFromEnvironment.trim().toLowerCase().contains("jsc")))
            return JSC;
         else
            return IHMC;
      }
   };

   public static final boolean launchFootstepPlannerModule = true;
   private static final String REAConfigurationFilePath = System.getProperty("user.home") + "/.ihmc/Configurations/defaultREAModuleConfiguration.txt";

   public static void startIHMCNetworkProcessor(ValkyrieRobotModel robotModel)
   {
      HumanoidNetworkProcessor networkProcessor = new HumanoidNetworkProcessor(robotModel, PubSubImplementation.FAST_RTPS);

      networkProcessor.setupKinematicsToolboxModule(false);
      networkProcessor.setupKinematicsStreamingToolboxModule(ValkyrieKinematicsStreamingToolboxModule.class, null, true);

      new ValkyrieAStarFootstepPlanner(robotModel).setupWithRos(PubSubImplementation.FAST_RTPS);
      if (launchFootstepPlannerModule)
         networkProcessor.setupFootstepPlanningToolboxModule(new ValkyrieAdaptiveSwingParameters());
      networkProcessor.setupWalkingPreviewModule(false);

      networkProcessor.setupBipedalSupportPlanarRegionPublisherModule();
      networkProcessor.setupHumanoidAvatarREAStateUpdater();
      networkProcessor.setupRosModule();

      ValkyrieSensorSuiteManager sensorModule = robotModel.getSensorSuiteManager();
      sensorModule.setEnableLidarScanPublisher(true);
      sensorModule.setEnableStereoVisionPointCloudPublisher(true);
      sensorModule.setEnableVideoPublisher(true);

      networkProcessor.setupSensorModule();

      sensorModule.getLidarScanPublisher().setPublisherPeriodInMillisecond(25L);
      sensorModule.getMultiSenseSensorManager().setVideoSettings(VideoControlSettings.configureJPEGServer(35, 20));
      sensorModule.getStereoVisionPointCloudPublisher().setRangeFilter(0.2, 2.5);
      sensorModule.getStereoVisionPointCloudPublisher().setSelfCollisionFilter(robotModel.getCollisionBoxProvider());
      sensorModule.getStereoVisionPointCloudPublisher().setPublisherPeriodInMillisecond(750L);
      sensorModule.getStereoVisionPointCloudPublisher().setMaximumNumberOfPoints(500000);
      sensorModule.getStereoVisionPointCloudPublisher().setMinimumResolution(0.005);

      LogTools.info("ROS_MASTER_URI=" + networkProcessor.getOrCreateRosURI());

      networkProcessor.setupShutdownHook();
      networkProcessor.start();
   }
   
   public static EnumMap<ValkyrieNetworkProcessorParameters, Boolean> iniToParameters(File file)
   {
      EnumMap<ValkyrieNetworkProcessorParameters, Boolean> parameters = new EnumMap<ValkyrieNetworkProcessorParameters, Boolean>(
            ValkyrieNetworkProcessorParameters.class);

      for (ValkyrieNetworkProcessorParameters key : ValkyrieNetworkProcessorParameters.values()) {
         parameters.put(key, key.getDefaultValue());
      }
      
      if (file.exists() && file.isFile()) {
         LogTools.info("Found parameters file at " + file.getAbsolutePath());
         try {
            Properties properties = new Properties();
            FileInputStream stream = new FileInputStream(file);
            properties.load(stream);
            for (ValkyrieNetworkProcessorParameters key : ValkyrieNetworkProcessorParameters.values()) {
               String keyString = key.toString();
               if (properties.containsKey(keyString)) {
                  parameters.put(key, Boolean.valueOf(properties.getProperty(keyString)));
                  LogTools.info("Valkyrie Network Processor Setting " + keyString + " to " + parameters.get(key).toString());
               }
            }
            stream.close();
         } catch (IOException e) {
            System.err.println(
                  "Valkyrie network processor parameter file " + file.getAbsolutePath() + "exists but cannot be loaded. See stack trace.");
            e.printStackTrace();
         }
      } else {
         LogTools.warn("Valkyrie network processor parameter file " + file.getAbsolutePath() + " does not exist.");
      }
      return parameters;
   }

   public static void startJSCNetworkProcessor(ValkyrieRobotModel robotModel)
   {
      HumanoidNetworkProcessor networkProcessor = new HumanoidNetworkProcessor(robotModel, PubSubImplementation.FAST_RTPS);
      final String valkyrieNetworkProcessorConfig = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "ValkyrieNetworkProcessorModuleConfig.ini";
      
      File configurationFile = new File(System.getProperty("us.ihmc.networkParameterFile", valkyrieNetworkProcessorConfig)).getAbsoluteFile();
      LogTools.info("Looking for network processor module configuration in " + configurationFile.getAbsolutePath());
      EnumMap<ValkyrieNetworkProcessorParameters, Boolean> parameters = iniToParameters(configurationFile);
      
      if (parameters.get(ValkyrieNetworkProcessorParameters.start_kinematics_streaming_toolbox)) {

         networkProcessor.setupKinematicsStreamingToolboxModule(ValkyrieKinematicsStreamingToolboxModule.class, null,
               true);
      }

      if (parameters.get(ValkyrieNetworkProcessorParameters.start_force_estimation)) {
         new ValkyrieExternalForceEstimationModule(robotModel, false, PubSubImplementation.FAST_RTPS);
      }

      if (parameters.get(ValkyrieNetworkProcessorParameters.start_footstep_planning)) {
         new ValkyrieAStarFootstepPlanner(robotModel).setupWithRos(PubSubImplementation.FAST_RTPS);
         networkProcessor.setupFootstepPlanningToolboxModule(new ValkyrieAdaptiveSwingParameters());
      }

      if (parameters.get(ValkyrieNetworkProcessorParameters.start_walking_preview)) {
         networkProcessor.setupWalkingPreviewModule(false);
      }

      if (parameters.get(ValkyrieNetworkProcessorParameters.start_rea)) {
         networkProcessor.setupRobotEnvironmentAwerenessModule(REAConfigurationFilePath);
         networkProcessor.setupBipedalSupportPlanarRegionPublisherModule();
         networkProcessor.setupHumanoidAvatarREAStateUpdater();
      }

       // Always required for Valkyrie
       networkProcessor.setupRosModule();

       if (parameters.get(ValkyrieNetworkProcessorParameters.start_sensor_processing)) {
          ValkyrieSensorSuiteManager sensorSuiteManager = robotModel.getSensorSuiteManager();
          sensorSuiteManager.setEnableLidarScanPublisher(parameters.get(ValkyrieNetworkProcessorParameters.start_lidar));
          sensorSuiteManager.setEnableStereoVisionPointCloudPublisher(parameters.get(ValkyrieNetworkProcessorParameters.start_stereo_vision_pointcloud));
          
          sensorSuiteManager.setEnableVideoPublisher(false);
          networkProcessor.setupSensorModule();
       }

      LogTools.info("ROS_MASTER_URI=" + networkProcessor.getOrCreateRosURI());

      networkProcessor.setupShutdownHook();
      networkProcessor.start();
   }

   public static void main(String[] args) throws JSAPException
   {
      ValkyrieRobotVersion robotVersion = ValkyrieRosControlController.VERSION;
      LogTools.info("Valkyrie robot version: " + robotVersion);
      NetworkProcessorVersion netProcVersion = NetworkProcessorVersion.fromEnvironment();
      LogTools.info("Configuring network processor for " + netProcVersion.name());

      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, robotVersion);

      switch (netProcVersion)
      {
         case IHMC:
            startIHMCNetworkProcessor(robotModel);
            break;
         case JSC:
            startJSCNetworkProcessor(robotModel);
            break;
         default:
            throw new IllegalStateException("Unhandled version: " + netProcVersion);
      }
   }
}
