package us.ihmc.valkyrie;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessor;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlanner;
import us.ihmc.valkyrie.sensors.ValkyrieSensorSuiteManager;

public class ValkyrieNetworkProcessor
{
   private enum NetworkProcessorVersion
   {
      IHMC, JSC;

      public static NetworkProcessorVersion fromEnvironment()
      {
         String rosDistro = System.getenv("NETWORK_PROCESSOR_VERSION");

         if (rosDistro != null && (rosDistro.trim().toLowerCase().contains("jsc")))
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

      new ValkyrieAStarFootstepPlanner(robotModel).setupWithRos(PubSubImplementation.FAST_RTPS);
      if (launchFootstepPlannerModule)
         networkProcessor.setupFootstepPlanningToolboxModule(false);
      networkProcessor.setupWalkingPreviewModule(false);

      networkProcessor.setupBipedalSupportPlanarRegionPublisherModule();
      networkProcessor.setupHumanoidAvatarREAStateUpdater();
      networkProcessor.setupRosModule();
      ValkyrieSensorSuiteManager sensorSuiteManager = (ValkyrieSensorSuiteManager) networkProcessor.setupSensorModule();
      sensorSuiteManager.setEnableLidarScanPublisher(true);
      sensorSuiteManager.setEnableStereoVisionPointCloudPublisher(false);
      sensorSuiteManager.setEnableVideoPublisher(true);

      LogTools.info("ROS_MASTER_URI=" + networkProcessor.getRosURI());

      networkProcessor.setupShutdownHook();
      networkProcessor.start();
   }

   public static void startJSCNetworkProcessor(ValkyrieRobotModel robotModel)
   {
      HumanoidNetworkProcessor networkProcessor = new HumanoidNetworkProcessor(robotModel, PubSubImplementation.FAST_RTPS);

      networkProcessor.setupKinematicsStreamingToolboxModule(ValkyrieKinematicsStreamingToolboxModule.class, null, true);

      new ValkyrieAStarFootstepPlanner(robotModel).setupWithRos(PubSubImplementation.FAST_RTPS);
      if (launchFootstepPlannerModule)
         networkProcessor.setupFootstepPlanningToolboxModule(false);
      networkProcessor.setupWalkingPreviewModule(false);

      networkProcessor.setupRobotEnvironmentAwerenessModule(REAConfigurationFilePath);
      networkProcessor.setupBipedalSupportPlanarRegionPublisherModule();
      networkProcessor.setupHumanoidAvatarREAStateUpdater();
      networkProcessor.setupRosModule();
      ValkyrieSensorSuiteManager sensorSuiteManager = (ValkyrieSensorSuiteManager) networkProcessor.setupSensorModule();
      sensorSuiteManager.setEnableLidarScanPublisher(true);
      sensorSuiteManager.setEnableStereoVisionPointCloudPublisher(false);
      sensorSuiteManager.setEnableVideoPublisher(false);

      LogTools.info("ROS_MASTER_URI=" + networkProcessor.getRosURI());

      networkProcessor.setupShutdownHook();
      networkProcessor.start();
   }

   /**
    * Valid arguments are:
    * <ul>
    * <li>Valkyrie robot model, example: <tt>--model="DEFAULT"</tt> see {@link ValkyrieRobotVersion}
    * for available options.
    * </ul>
    */
   public static void main(String[] args) throws JSAPException
   {

      JSAP jsap = new JSAP();

      FlaggedOption robotModelFlag = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(false)
                                                                    .setStringParser(JSAP.STRING_PARSER);

      jsap.registerParameter(robotModelFlag);

      JSAPResult config = jsap.parse(args);

      ValkyrieRobotVersion robotVersion = ValkyrieRobotVersion.DEFAULT;
      NetworkProcessorVersion netProcVersion = NetworkProcessorVersion.fromEnvironment();
      LogTools.info("Configuring network processor for " + netProcVersion.name());

      if (config.success())
      {
         String robotModelArg = config.getString(robotModelFlag.getID());
         if (robotModelArg != null)
         {
            try
            {
               robotVersion = ValkyrieRobotVersion.valueOf(robotModelArg);
            }
            catch (IllegalArgumentException e)
            {
               LogTools.error("Invalid robotModel argument: " + robotModelArg + ", using DEFAULT.");
               robotVersion = ValkyrieRobotVersion.DEFAULT;
            }
         }
      }

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
