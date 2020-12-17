package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;
import us.ihmc.ros2.ROS2Node;

import java.io.File;
import java.nio.file.Paths;

import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.FAST_RTPS;

public class AtlasRealsenseSLAMRemoteLauncher
{
   private static final String SLAM_CONFIGURATION_FILE_NAME = "atlasSLAMModuleConfiguration.txt";
   private static final String SEGMENTATION_CONFIGURATION_FILE_NAME = "atlasSegmentationModuleConfiguration.txt";
   private final PubSubImplementation pubSubImplementation = FAST_RTPS;

   private ROS2Node ros2Node;
   private Messager slamMessager;
   private Messager segmentationMessager;

   private AtlasSLAMModule slamModule;
   private PlanarSegmentationModule segmentationModule;

   public AtlasRealsenseSLAMRemoteLauncher()
   {
      ExceptionTools.handle(this::setup, DefaultExceptionHandler.PRINT_STACKTRACE);
   }

   public void setup() throws Exception
   {
      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);

      ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, ROS2Tools.REA_NODE_NAME);

//      slamMessager = KryoMessager.createServer(SLAMModuleAPI.API, NetworkPorts.SLAM_MODULE_UI_PORT.getPort(), "SLAMModule", 5);
//      ThreadTools.startAThread(() -> ExceptionTools.handle(slamMessager::startMessager, DefaultExceptionHandler.RUNTIME_EXCEPTION), "KryoStarter");

      slamMessager = new SharedMemoryMessager(SLAMModuleAPI.API);
      slamMessager.startMessager();

//      segmentationMessager = KryoMessager.createServer(SegmentationModuleAPI.API, NetworkPorts.PLANAR_SEGMENTATION_UI_PORT.getPort(), "SegmentationModule", 5);
//      ThreadTools.startAThread(() -> ExceptionTools.handle(segmentationMessager::startMessager, DefaultExceptionHandler.RUNTIME_EXCEPTION), "KryoStarter");

      segmentationMessager = new SharedMemoryMessager(SegmentationModuleAPI.API);
      segmentationMessager.startMessager();

      File slamConfigurationFile = Paths.get(System.getProperty("user.home"), ".ihmc", SLAM_CONFIGURATION_FILE_NAME).toFile();
      File segmentationConfigurationFile = Paths.get(System.getProperty("user.home"), ".ihmc", SEGMENTATION_CONFIGURATION_FILE_NAME).toFile();

      slamModule = new AtlasSLAMModule(ros2Node, slamMessager, drcRobotModel, slamConfigurationFile);
      segmentationModule = PlanarSegmentationModule.createIntraprocessModule(segmentationConfigurationFile, ros2Node, segmentationMessager);

      slamModule.attachOcTreeConsumer(segmentationModule);

      new IHMCROS2Callback<>(ros2Node, SLAMModuleAPI.SHUTDOWN, message ->
      {
         LogTools.info("Received SHUTDOWN. Shutting down...");
         stop();
      });

      slamModule.start();
      segmentationModule.start();
   }

   public void stop()
   {
      ThreadTools.sleepSeconds(2.0);

      ExceptionTools.handle(() -> slamMessager.closeMessager(), DefaultExceptionHandler.PRINT_STACKTRACE);
      ExceptionTools.handle(() -> segmentationMessager.closeMessager(), DefaultExceptionHandler.PRINT_STACKTRACE);

      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new AtlasRealsenseSLAMRemoteLauncher();
   }
}
