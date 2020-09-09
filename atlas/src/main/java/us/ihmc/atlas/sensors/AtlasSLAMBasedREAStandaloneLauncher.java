package us.ihmc.atlas.sensors;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import javafx.application.Platform;
import javafx.stage.Stage;
import std_msgs.msg.dds.Empty;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.behaviors.AtlasLookAndStepBehaviorUIAndModule;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.rtps.impl.fastRTPS.FastRTPSDomain;
import us.ihmc.tools.io.WorkspacePathTools;
import us.ihmc.tools.processManagement.JavaProcessManager;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.*;

public class AtlasSLAMBasedREAStandaloneLauncher
{
   private static boolean launchSegmentation = true;

   private static final String SLAM_CONFIGURATION_FILE_NAME = "atlasSLAMModuleConfiguration.txt";
   private static final String SEGMENTATION_CONFIGURATION_FILE_NAME = "atlasSegmentationModuleConfiguration.txt";
   private final boolean spawnSegmentationUI;
   private final boolean spawnSLAMUI;
   private final DomainFactory.PubSubImplementation pubSubImplementation;

   private ROS2Node ros2Node;
   private Messager slamMessager;
   private Messager segmentationMessager;
   private SLAMBasedEnvironmentAwarenessUI ui;
   private AtlasSLAMModule module;

   private PlanarSegmentationUI planarSegmentationUI;
   private PlanarSegmentationModule segmentationModule;

   public AtlasSLAMBasedREAStandaloneLauncher(boolean spawnUIs, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this(spawnUIs, spawnUIs, pubSubImplementation);
   }
   
   public AtlasSLAMBasedREAStandaloneLauncher(boolean spawnSegmentationUI, boolean spawnSLAMUI, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this.spawnSegmentationUI = spawnSegmentationUI;
      this.spawnSLAMUI = spawnSLAMUI;

      this.pubSubImplementation = pubSubImplementation;

      Runnable setup = () -> ExceptionTools.handle(this::setup, DefaultExceptionHandler.PRINT_STACKTRACE);
      if (spawnSegmentationUI || spawnSLAMUI)
      {
         JavaFXApplicationCreator.createAJavaFXApplication();
         Platform.runLater(setup);
      }
      else
      {
         setup.run();
      }
   }

   public void setup() throws Exception
   {
      Stage primaryStage = null;
      if (spawnSLAMUI)
      {
         primaryStage = new Stage();
      }

      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);

      RobotContactPointParameters<RobotSide> contactPointParameters = drcRobotModel.getContactPointParameters();
      SideDependentList<List<Point2D>> defaultContactPoints = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         defaultContactPoints.put(side, contactPointParameters.getControllerFootGroundContactPoints().get(side));
      }

      ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, ROS2Tools.REA_NODE_NAME);

      slamMessager = spawnSLAMUI ? new SharedMemoryJavaFXMessager(SLAMModuleAPI.API) : new SharedMemoryMessager(SLAMModuleAPI.API);
      slamMessager.startMessager();

      if (launchSegmentation)
      {
         segmentationMessager = spawnSegmentationUI ? new SharedMemoryJavaFXMessager(SegmentationModuleAPI.API) : new SharedMemoryMessager(SegmentationModuleAPI.API);
         segmentationMessager.startMessager();
      }

      if (spawnSLAMUI)
      {
         ui = SLAMBasedEnvironmentAwarenessUI.creatIntraprocessUI(slamMessager, primaryStage, defaultContactPoints);
      }
      Path slamConfigurationFilePath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
      slamConfigurationFilePath = Paths.get(slamConfigurationFilePath.toString(), "/atlas/src/main/resources/" + SLAM_CONFIGURATION_FILE_NAME);
      LogTools.info("Loading configuration file: {}", slamConfigurationFilePath.toAbsolutePath().normalize());
      module = AtlasSLAMModule.createIntraprocessModule(ros2Node, drcRobotModel, slamMessager, slamConfigurationFilePath);

      Stage secondStage = null;
      if (launchSegmentation)
      {
         if (spawnSegmentationUI)
         {
            secondStage = new Stage();
            planarSegmentationUI = PlanarSegmentationUI.createIntraprocessUI(segmentationMessager, secondStage);
         }

         Path segmentationConfigurationFilePath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
         segmentationConfigurationFilePath = Paths.get(segmentationConfigurationFilePath.toString(), "/atlas/src/main/resources/" + SEGMENTATION_CONFIGURATION_FILE_NAME);

         segmentationModule = PlanarSegmentationModule.createIntraprocessModule(segmentationConfigurationFilePath, ros2Node, segmentationMessager);
         module.attachOcTreeConsumer(segmentationModule);
      }

      if (spawnSLAMUI)
      {
//         primaryStage.setOnCloseRequest(event -> stop());
         ui.show();

      }
      if (spawnSegmentationUI && launchSegmentation)
      {
//         secondStage.setOnCloseRequest(event -> stop());
         planarSegmentationUI.show();
      }

      new IHMCROS2Callback<>(ros2Node, SLAMModuleAPI.SHUTDOWN, message ->
      {
         LogTools.info("Received SHUTDOWN. Shutting down...");
         stop();
      });

      module.start();
      if (segmentationModule != null)
         segmentationModule.start();
   }

   public void stop()
   {
      ThreadTools.sleepSeconds(2.0);
      LogTools.info("Stopping");

      Platform.exit();
      LogTools.info("Stopping");
//      if (spawnSLAMUI) ui.stop();
//      module.stop();

      ExceptionTools.handle(() -> slamMessager.closeMessager(), DefaultExceptionHandler.PRINT_STACKTRACE);
      LogTools.info("Stopping");

      if (launchSegmentation)
      {
//         if (spawnSegmentationUI)
//            planarSegmentationUI.stop();
//         segmentationModule.stop();
         ExceptionTools.handle(() -> segmentationMessager.closeMessager(), DefaultExceptionHandler.PRINT_STACKTRACE);
      }
      LogTools.info("Stopping");

//      ros2Node.destroy();

//      ThreadTools.startAThread(() ->
//                               {
//                                  FastRTPSDomain.getInstance().stopAll();
//                                  LogTools.info("Stopped everything");
//                               }, "StopAllROS2");

      ThreadTools.sleepSeconds(1.0);

      // Halting in a daemon thread only has to do so if everything else failed.
//      ThreadTools.startAsDaemon(() ->
//      {
//         ThreadTools.sleepSeconds(2.0);
//         Runtime.getRuntime().halt(1);
//      }, "EventuallyHalt");
   }

   public static void main(String[] args)
   {
      JavaProcessManager manager = new JavaProcessManager();
      manager.runOrRegister("AtlasSLAMBasedREA", () -> new AtlasSLAMBasedREAStandaloneLauncher(true, true, FAST_RTPS));
      ArrayList<Process> processes = manager.spawnProcesses(AtlasSLAMBasedREAStandaloneLauncher.class, args);

      ROS2Node ros2Node = ROS2Tools.createROS2Node(FAST_RTPS, "test_node");
      new IHMCROS2Callback<>(ros2Node, SLAMModuleAPI.SHUTDOWN, message ->
      {
         LogTools.info("Received SHUTDOWN. Shutting down...");

         ThreadTools.startAsDaemon(() ->
                                   {
                                      ThreadTools.sleepSeconds(2.0);
                                      for (Process process : processes)
                                      {
                                         LogTools.info("Destoying process  forcibly");
                                         process.destroyForcibly();
                                      }
                                      ros2Node.destroy();
                                   }, "DestroyThread");
      });
   }
}
