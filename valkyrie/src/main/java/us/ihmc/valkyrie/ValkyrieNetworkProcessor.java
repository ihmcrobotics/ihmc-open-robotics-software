package us.ihmc.valkyrie;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessor;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieAdaptiveSwingParameters;
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

      networkProcessor.setupKinematicsStreamingToolboxModule(ValkyrieKinematicsStreamingToolboxModule.class, null, true);

      if (launchFootstepPlannerModule)
         networkProcessor.setupFootstepPlanningToolboxModule(new ValkyrieAdaptiveSwingParameters());
      networkProcessor.setupWalkingPreviewModule(false);

      networkProcessor.setupBipedalSupportPlanarRegionPublisherModule();
      networkProcessor.setupHumanoidAvatarREAStateUpdater();
      networkProcessor.setupRosModule();

      ValkyrieSensorSuiteManager sensorSuiteManager = robotModel.getSensorSuiteManager();
      sensorSuiteManager.setEnableLidarScanPublisher(true);
      sensorSuiteManager.setEnableStereoVisionPointCloudPublisher(false);
      sensorSuiteManager.setEnableVideoPublisher(true);
      networkProcessor.setupSensorModule();

      LogTools.info("ROS_MASTER_URI=" + networkProcessor.getOrCreateRosURI());

      networkProcessor.setupShutdownHook();
      networkProcessor.start();
   }

   public static void startJSCNetworkProcessor(ValkyrieRobotModel robotModel)
   {
      HumanoidNetworkProcessor networkProcessor = new HumanoidNetworkProcessor(robotModel, PubSubImplementation.FAST_RTPS);

      networkProcessor.setupKinematicsStreamingToolboxModule(ValkyrieKinematicsStreamingToolboxModule.class, null, true);

      if (launchFootstepPlannerModule)
         networkProcessor.setupFootstepPlanningToolboxModule(new ValkyrieAdaptiveSwingParameters());
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
