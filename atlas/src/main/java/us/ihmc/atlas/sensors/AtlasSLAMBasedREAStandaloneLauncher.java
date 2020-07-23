package us.ihmc.atlas.sensors;

import java.util.List;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class AtlasSLAMBasedREAStandaloneLauncher
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultSegmentationModuleConfiguration.txt";
   private final boolean spawnUIs;

   private Messager slamMessager;
   private Messager segmentationMessager;
   private SLAMBasedEnvironmentAwarenessUI ui;
   private AtlasSLAMModule module;

   private PlanarSegmentationUI planarSegmentationUI;
   private PlanarSegmentationModule segmentationModule;

   public AtlasSLAMBasedREAStandaloneLauncher(boolean spawnUIs)
   {
      this.spawnUIs = spawnUIs;

      JavaFXApplicationCreator.createAJavaFXApplication();

      Platform.runLater(() -> ExceptionTools.handle(this::setup, DefaultExceptionHandler.PRINT_STACKTRACE));
   }

   public void setup() throws Exception
   {
      Stage primaryStage = new Stage();

      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);

      RobotContactPointParameters<RobotSide> contactPointParameters = drcRobotModel.getContactPointParameters();
      SideDependentList<List<Point2D>> defaultContactPoints = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         defaultContactPoints.put(side, contactPointParameters.getControllerFootGroundContactPoints().get(side));
      }

      slamMessager = new SharedMemoryJavaFXMessager(SLAMModuleAPI.API);
      slamMessager.startMessager();

      segmentationMessager = new SharedMemoryJavaFXMessager(SegmentationModuleAPI.API);
      segmentationMessager.startMessager();

      if (spawnUIs)
      {
         ui = SLAMBasedEnvironmentAwarenessUI.creatIntraprocessUI(slamMessager, primaryStage, defaultContactPoints);
      }
      module = AtlasSLAMModule.createIntraprocessModule(drcRobotModel, slamMessager);

      Stage secondStage = null;
      if (spawnUIs)
      {
         secondStage = new Stage();
         planarSegmentationUI = PlanarSegmentationUI.createIntraprocessUI(segmentationMessager, secondStage);
      }
      segmentationModule = PlanarSegmentationModule.createIntraprocessModule(REACommunicationProperties.inputTopic,
                                                                             REACommunicationProperties.subscriberCustomRegionsTopicName,
                                                                             ROS2Tools.REALSENSE_SLAM_REGIONS,
                                                                             MODULE_CONFIGURATION_FILE_NAME,
                                                                             segmentationMessager);
      module.attachOcTreeConsumer(segmentationModule);

      if (spawnUIs)
      {
         primaryStage.setOnCloseRequest(event -> ExceptionTools.handle(() -> stop(), DefaultExceptionHandler.PRINT_STACKTRACE));
         secondStage.setOnCloseRequest(event -> ExceptionTools.handle(() -> stop(), DefaultExceptionHandler.PRINT_STACKTRACE));

         ui.show();
         planarSegmentationUI.show();
      }

      module.start();
      segmentationModule.start();
   }

   public void stop() throws Exception
   {
      ui.stop();
      module.stop();

      planarSegmentationUI.stop();
      segmentationModule.stop();

      slamMessager.closeMessager();
      segmentationMessager.closeMessager();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      new AtlasSLAMBasedREAStandaloneLauncher(true);
   }
}
