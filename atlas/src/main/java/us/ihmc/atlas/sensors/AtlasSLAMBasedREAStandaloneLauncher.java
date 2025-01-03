package us.ihmc.atlas.sensors;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaFXToolkit.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.tools.io.WorkspacePathTools;
import us.ihmc.tools.processManagement.JavaProcessManager;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

public class AtlasSLAMBasedREAStandaloneLauncher
{
   private static boolean launchSegmentation = true;

   private static final String SLAM_CONFIGURATION_FILE_NAME = "atlasSLAMModuleConfiguration.txt";
   private static final String SEGMENTATION_CONFIGURATION_FILE_NAME = "atlasSegmentationModuleConfiguration.txt";
   private final boolean spawnSegmentationUI;
   private final boolean spawnSLAMUI;

   private ROS2Node ros2Node;
   private Messager slamMessager;
   private Messager segmentationMessager;
   private SLAMBasedEnvironmentAwarenessUI ui;
   private AtlasSLAMModule module;

   private PlanarSegmentationUI planarSegmentationUI;
   private PlanarSegmentationModule segmentationModule;

   public AtlasSLAMBasedREAStandaloneLauncher(boolean spawnUIs)
   {
      this(spawnUIs, spawnUIs);
   }
   
   public AtlasSLAMBasedREAStandaloneLauncher(boolean spawnSegmentationUI, boolean spawnSLAMUI)
   {
      this.spawnSegmentationUI = spawnSegmentationUI;
      this.spawnSLAMUI = spawnSLAMUI;

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

      ros2Node = new ROS2NodeBuilder().build(PerceptionAPI.REA_NODE_NAME);

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
      if (slamConfigurationFilePath != null)
      {
         slamConfigurationFilePath = Paths.get(slamConfigurationFilePath.toString(), "/atlas/src/main/resources/" + SLAM_CONFIGURATION_FILE_NAME)
                                          .toAbsolutePath()
                                          .normalize();
         LogTools.info("Loading configuration file: {}", slamConfigurationFilePath);
      }
      else
      {
         LogTools.info("Running from JAR. Saving configuration disabled. {}");
      }
      module = AtlasSLAMModule.createIntraprocessModule(ros2Node,
                                                        drcRobotModel,
                                                        slamMessager,
                                                        slamConfigurationFilePath == null ? null : slamConfigurationFilePath.toFile());

      Stage secondStage = null;
      if (launchSegmentation)
      {
         if (spawnSegmentationUI)
         {
            secondStage = new Stage();
            planarSegmentationUI = PlanarSegmentationUI.createIntraprocessUI(segmentationMessager, secondStage);
         }

         Path segmentationConfigurationFilePath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
         if (segmentationConfigurationFilePath != null)
         {
            segmentationConfigurationFilePath = Paths.get(segmentationConfigurationFilePath.toString(),
                                                          "/atlas/src/main/resources/" + SEGMENTATION_CONFIGURATION_FILE_NAME);
         }

         segmentationModule
               = PlanarSegmentationModule.createIntraprocessModule(segmentationConfigurationFilePath == null ? null : segmentationConfigurationFilePath.toFile(),
                                                                   ros2Node,
                                                                   segmentationMessager);
         module.attachOcTreeConsumer(segmentationModule);
      }

      if (spawnSLAMUI)
      {
         ui.show();
      }
      if (spawnSegmentationUI && launchSegmentation)
      {
         planarSegmentationUI.show();
      }

      ros2Node.createSubscription2(SLAMModuleAPI.SHUTDOWN, message ->
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

      ExceptionTools.handle(() -> slamMessager.closeMessager(), DefaultExceptionHandler.PRINT_STACKTRACE);

      if (launchSegmentation)
      {
         ExceptionTools.handle(() -> segmentationMessager.closeMessager(), DefaultExceptionHandler.PRINT_STACKTRACE);
      }

      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      JavaProcessManager manager = new JavaProcessManager();
      manager.runOrRegister("AtlasSLAMBasedREA", () -> new AtlasSLAMBasedREAStandaloneLauncher(true, true));
      ArrayList<Process> processes = manager.spawnProcesses(AtlasSLAMBasedREAStandaloneLauncher.class, args);

      ROS2Node ros2Node = new ROS2NodeBuilder().build("test_node");
      ros2Node.createSubscription2(SLAMModuleAPI.SHUTDOWN, message ->
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
