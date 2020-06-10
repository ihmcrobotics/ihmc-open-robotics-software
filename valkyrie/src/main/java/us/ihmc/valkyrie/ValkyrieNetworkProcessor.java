package us.ihmc.valkyrie;

import com.martiansoftware.jsap.JSAPException;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessor;
import us.ihmc.communication.producers.VideoControlSettings;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.externalForceEstimation.ValkyrieExternalForceEstimationModule;
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

   /** Whether or not to start the footstep planner when running the IHMC network processor. */
   private static final boolean ihmc_launchFootstepPlannerModule = false;

   private static final String REAConfigurationFilePath = System.getProperty("user.home") + "/.ihmc/Configurations/defaultREAModuleConfiguration.txt";

   public static void startIHMCNetworkProcessor(ValkyrieRobotModel robotModel)
   {
      HumanoidNetworkProcessor networkProcessor = new HumanoidNetworkProcessor(robotModel, PubSubImplementation.FAST_RTPS);

      networkProcessor.setupKinematicsToolboxModule(false);
      networkProcessor.setupKinematicsStreamingToolboxModule(ValkyrieKinematicsStreamingToolboxModule.class, null, true);

      if (ihmc_launchFootstepPlannerModule)
         networkProcessor.setupFootstepPlanningToolboxModule();
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

   public static void startJSCNetworkProcessor(ValkyrieRobotModel robotModel)
   {
      HumanoidNetworkProcessor networkProcessor = new HumanoidNetworkProcessor(robotModel, PubSubImplementation.FAST_RTPS);

      networkProcessor.setupKinematicsStreamingToolboxModule(ValkyrieKinematicsStreamingToolboxModule.class, null, true);

      new ValkyrieExternalForceEstimationModule(robotModel, false, PubSubImplementation.FAST_RTPS);
      networkProcessor.setupFootstepPlanningToolboxModule();
      networkProcessor.setupWalkingPreviewModule(false);

      networkProcessor.setupRobotEnvironmentAwerenessModule(REAConfigurationFilePath);
      networkProcessor.setupBipedalSupportPlanarRegionPublisherModule();
      networkProcessor.setupHumanoidAvatarREAStateUpdater();
      networkProcessor.setupRosModule();

      ValkyrieSensorSuiteManager sensorSuiteManager = robotModel.getSensorSuiteManager();
      sensorSuiteManager.setEnableLidarScanPublisher(true);
      sensorSuiteManager.setEnableStereoVisionPointCloudPublisher(false);
      sensorSuiteManager.setEnableVideoPublisher(false);
      networkProcessor.setupSensorModule();

      LogTools.info("ROS_MASTER_URI=" + networkProcessor.getOrCreateRosURI());

      networkProcessor.setupShutdownHook();
      networkProcessor.start();
   }

   public static boolean isFootstepPlanningModuleStarted()
   {
      if (NetworkProcessorVersion.fromEnvironment() == NetworkProcessorVersion.IHMC)
      {
         return ihmc_launchFootstepPlannerModule;
      }
      else
      {
         return true;
      }
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
