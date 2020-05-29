package us.ihmc.atlas;

import com.martiansoftware.jsap.*;

import us.ihmc.atlas.sensors.AtlasSensorSuiteManager;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessor;
import us.ihmc.communication.producers.VideoControlSettings;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class AtlasNetworkProcessor
{
   private static final Application DEFAULT = AtlasNetworkProcessor::defaultNetworkProcessor;
   private static final Application VR = AtlasNetworkProcessor::vrNetworkProcessor;
   private static final Application MINIMAL = AtlasNetworkProcessor::minimalNetworkProcessor;

   private static final Application APPLICATION = DEFAULT;

   public static void main(String[] args) throws JSAPException
   {
      JSAP jsap = new JSAP();

      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);

      Switch runningOnRealRobot = new Switch("runningOnRealRobot").setLongFlag("realRobot");
      Switch runningOnGazebo = new Switch("runningOnGazebo").setLongFlag("gazebo");

      FlaggedOption leftHandHost = new FlaggedOption("leftHandHost").setLongFlag("lefthand").setShortFlag('l').setRequired(false)
                                                                    .setStringParser(JSAP.STRING_PARSER);
      FlaggedOption rightHandHost = new FlaggedOption("rightHandHost").setLongFlag("righthand").setShortFlag('r').setRequired(false)
                                                                      .setStringParser(JSAP.STRING_PARSER);

      robotModel.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      jsap.registerParameter(robotModel);

      jsap.registerParameter(runningOnRealRobot);
      jsap.registerParameter(runningOnGazebo);
      jsap.registerParameter(leftHandHost);
      jsap.registerParameter(rightHandHost);

      JSAPResult config = jsap.parse(args);

      if (!config.success())
      {
         System.err.println("Invalid parameters");
         System.out.println(jsap.getHelp());
         return;
      }

      RobotTarget target;
      AtlasRobotModel model;

      try
      {
         if (config.getBoolean(runningOnRealRobot.getID()))
         {
            target = RobotTarget.REAL_ROBOT;
         }
         else if (config.getBoolean(runningOnGazebo.getID()))
         {
            target = RobotTarget.GAZEBO;
         }
         else
         {
            target = RobotTarget.SCS;
         }
         model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), target, true);
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect robot model " + config.getString("robotModel"));
         System.out.println(jsap.getHelp());
         return;
      }

      LogTools.info("Selected model: {}", model);

      HumanoidNetworkProcessor networkProcessor = new HumanoidNetworkProcessor(model, PubSubImplementation.FAST_RTPS);
      LogTools.info("ROS_MASTER_URI = " + networkProcessor.getOrCreateRosURI());

      APPLICATION.setup(args, model, networkProcessor);

      networkProcessor.setupShutdownHook();
      networkProcessor.start();
   }

   private interface Application
   {
      void setup(String[] args, AtlasRobotModel robotModel, HumanoidNetworkProcessor networkProcessor);
   }

   private static void defaultNetworkProcessor(String[] args, AtlasRobotModel robotModel, HumanoidNetworkProcessor networkProcessor)
   {
      networkProcessor.setupRosModule();
      networkProcessor.setupBipedalSupportPlanarRegionPublisherModule();
      networkProcessor.setupHumanoidAvatarREAStateUpdater();
      networkProcessor.setupKinematicsToolboxModule(false);
      
      AtlasSensorSuiteManager sensorModule = robotModel.getSensorSuiteManager();
      networkProcessor.setupSensorModule();
      sensorModule.getLidarScanPublisher().setRangeFilter(0.2, 8.0);
      sensorModule.getLidarScanPublisher().setPublisherPeriodInMillisecond(25L);
      sensorModule.getMultiSenseSensorManager().setVideoSettings(VideoControlSettings.configureJPEGServer(35, 15));
      
      networkProcessor.setupKinematicsStreamingToolboxModule(AtlasKinematicsStreamingToolboxModule.class, args, false);
      networkProcessor.setupBehaviorModule(false, false, 0);
   }

   private static void vrNetworkProcessor(String[] args, AtlasRobotModel robotModel, HumanoidNetworkProcessor networkProcessor)
   {
      networkProcessor.setupRosModule();
      networkProcessor.setupBipedalSupportPlanarRegionPublisherModule();
      networkProcessor.setupHumanoidAvatarREAStateUpdater();

      AtlasSensorSuiteManager sensorModule = robotModel.getSensorSuiteManager();
      sensorModule.setEnableDepthPointCloudPublisher(false);
      sensorModule.setEnableFisheyeCameraPublishers(false);
      sensorModule.setEnableLidarScanPublisher(true);
      sensorModule.setEnableStereoVisionPointCloudPublisher(true);
      sensorModule.setEnableVideoPublisher(true);
      networkProcessor.setupSensorModule();
      sensorModule.getLidarScanPublisher().setRangeFilter(0.2, 8.0);
      sensorModule.getLidarScanPublisher().setPublisherPeriodInMillisecond(25L);
      sensorModule.getMultisenseStereoVisionPointCloudPublisher().setRangeFilter(0.2, 2.5);
      sensorModule.getMultisenseStereoVisionPointCloudPublisher().setPublisherPeriodInMillisecond(1500L);
      sensorModule.getMultisenseStereoVisionPointCloudPublisher().setMaximumNumberOfPoints(200000);
      sensorModule.getMultiSenseSensorManager().setVideoSettings(VideoControlSettings.configureJPEGServer(25, 10));

      networkProcessor.setupKinematicsStreamingToolboxModule(AtlasKinematicsStreamingToolboxModule.class, args, false);
      networkProcessor.setupBehaviorModule(false, false, 0);
   }

   private static void minimalNetworkProcessor(String[] args, AtlasRobotModel robotModel, HumanoidNetworkProcessor networkProcessor)
   {
      networkProcessor.setupRosModule();
      networkProcessor.setupSensorModule();
   }
}